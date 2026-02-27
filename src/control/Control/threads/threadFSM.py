from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.control.Control.threads.allStates import BehaviorState, SignType, ObstacleZone, SpeedLimit
from src.utils.messages.allMessages import (
    ControlAction, LaneData, LidarObstacle, SignDetection, Location
)
import time

class threadFSM(ThreadWithStop):
    """
    The 'Brain' of the E-Wolf. It aggregates sensor inputs to manage 
    Behavioral States and sends commands to threadControl.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # --- STATE REGISTRY ---
        self.current_state = BehaviorState.IDLE
        
        # --- INPUT MEMORY ---
        self.lane_info = {"e_y": 0.0, "theta_e": 0.0, "reliability": 0.0}
        self.obstacle_info = {"distance": 2000.0, "reliability": 0.0}
        self.active_sign = {"type": None, "distance": 2000.0}
        self.current_target_speed = SpeedLimit.CITY_MIN.value
        self.current_state = BehaviorState.IDLE
        self.stop_timer_start = None
        self.stop_reason = None # Reason for stop "SIGN" or "PEDESTRIAN"
        
        self.subscribe()
        
        # Action Sender (The output of the FSM)
        self.controlSender = messageHandlerSender(self.queuesList, ControlAction)
        
        super(threadFSM, self).__init__(pause=0.01) # 100Hz Decision Loop

    def evaluate_obstacle_zone(self):
        """Maps raw distance to a logical zone."""
        distance = self.obstacle_info["distance"]
        reliability = self.obstacle_info["reliability"]

        # If sensor is unreliable, treat as DANGER for safety
        if reliability < 0.2: 
            return ObstacleZone.DANGER
        
        if distance < 250.0: # DANGER ZONE
            return ObstacleZone.DANGER
        elif distance < 600.0: # Warning zone
            return ObstacleZone.WARNING
        else:
            return ObstacleZone.CLEAR   #Clear zone

    def subscribe(self):
        """Initializes subscribers for all 'Sense' threads."""
        # Lane Data from threadLane
        self.laneSub = messageHandlerSubscriber(self.queuesList, LaneData, "lastOnly", True)
        # Obstacle Data from threadDetector (Lidar) 
        self.lidarSub = messageHandlerSubscriber(self.queuesList, LidarObstacle, "lastOnly", True)
        # Sign/Signal Data from threadSigns (YOLO/V2X)
        self.signSub = messageHandlerSubscriber(self.queuesList, SignDetection, "lastOnly", True)

    def update_inputs(self):
        """
        Polls all subscribers and updates internal memory.
        This represents the 'Sensing' phase of the FSM.
        """
        # --- Lane ---
        lane_data = self.laneSub.receive()
        if lane_data:
            self.lane_info['e_y'] = lane_data.get('e_y', 0.0)
            self.lane_info['theta_e'] = lane_data.get('theta_e', 0.0)
            self.lane_info['reliability'] = lane_data.get('reliability', 0.0)

        # --- Obstacle ---
        lidar_data = self.lidarSub.receive()
        if lidar_data:
            self.obstacle_info['distance'] = lidar_data.get('distance', 2000.0)
            self.obstacle_info['reliability'] = lidar_data.get('reliability', 0.0)

        # --- Signs ---
        sign_data = self.signSub.receive()
        if sign_data:
            self.active_sign['type'] = sign_data.get('type', None)
            self.active_sign['distance'] = sign_data.get('distance', 2000.0)

    def update_state(self):
        """Thinking Phase: Transition logic following FSM.pdf."""
        zone = self.evaluate_obstacle_zone()
        sign = self.active_sign['type']
        s_dist = self.active_sign['distance']

        # --- PRIORITY 1: EMERGENCY BRAKE --- 
        if zone == ObstacleZone.DANGER:
            if self.current_state != BehaviorState.EMERGENCY_BRAKE:
                self.previous_state = self.current_state
                self.current_state = BehaviorState.EMERGENCY_BRAKE
                self.stop_timer_start = None # Reset timers on emergency
            return

        if self.current_state == BehaviorState.EMERGENCY_BRAKE:
            if zone == ObstacleZone.CLEAR:
                self.current_state = self.previous_state
            return

        # --- PRIORITY 2: TRAFFIC RULES ---
        
        # IDLE -> LANE_FOLLOWING
        if self.current_state == BehaviorState.IDLE:
            self.current_state = BehaviorState.LANE_FOLLOWING

        # LANE_FOLLOWING Transitions
        elif self.current_state == BehaviorState.LANE_FOLLOWING:
            if (sign in (SignType.STOP, SignType.PARKING, SignType.CROSSWALK) and s_dist < 600) or \
               (zone == ObstacleZone.WARNING):
                self.current_state = BehaviorState.DECELERATING
            elif sign == SignType.PRIORITY and s_dist < 600:
                self.current_state = BehaviorState.INTERSECTION
            elif sign == SignType.HIGHWAY_ENTRY:
                self.current_state = BehaviorState.HIGHWAY_DRIVING
        
        # IN HIGHWAY
        elif self.current_state == BehaviorState.HIGHWAY_DRIVING:
            if sign == SignType.HIGHWAY_EXIT:
                self.current_state = BehaviorState.LANE_FOLLOWING

        # DECELERATING
        elif self.current_state == BehaviorState.DECELERATING:
            if sign == SignType.STOP and s_dist < 150:
                self.current_state = BehaviorState.STOP_ACTION
                self.stop_reason = "SIGN"
            elif sign == SignType.CROSSWALK and zone == ObstacleZone.DANGER:
                self.current_state = BehaviorState.STOP_ACTION
                self.stop_reason = "PEDESTRIAN"
            elif sign == SignType.PARKING and s_dist < 200:
                self.current_state = BehaviorState.PARKING_MANEUVER
            elif zone == ObstacleZone.CLEAR and s_dist > 900:
                self.current_state = BehaviorState.LANE_FOLLOWING

        # STOP_ACTION (Wait 3s) 
        elif self.current_state == BehaviorState.STOP_ACTION:
            if self.stop_timer_start is None:
                self.stop_timer_start = time.perf_counter()
            
            if time.perf_counter() - self.stop_timer_start >= 3.0:
                self.stop_timer_start = None
                # Branching based on reason 
                if self.stop_reason == "SIGN":
                    self.current_state = BehaviorState.INTERSECTION
                else: # PEDESTRIAN
                    self.current_state = BehaviorState.LANE_FOLLOWING
                self.stop_reason = None

        # EXITING INTERSECTION / PARKING
        elif self.current_state in (BehaviorState.INTERSECTION, BehaviorState.PARKING_MANEUVER):
            if self.lane_info['reliability'] > 0.8:
                self.current_state = BehaviorState.LANE_FOLLOWING

    def execute_behavior(self):
        """
        Sends the appropriate ControlAction based on the current state.
        """

        if self.current_state == BehaviorState.LANE_FOLLOWING:
            target_speed = SpeedLimit.CITY_MIN.value

        elif self.current_state == BehaviorState.HIGHWAY_DRIVING:
            target_speed = SpeedLimit.HIGHWAY_MIN.value

        elif self.current_state == BehaviorState.DECELERATING:
            target_speed = SpeedLimit.CITY_MIN.value * 0.5

        elif self.current_state in (
                BehaviorState.STOP_ACTION,
                BehaviorState.IDLE,
                BehaviorState.EMERGENCY_BRAKE):
            target_speed = 0.0

        else:
            target_speed = 0.0

        command = {
            "behavior": self.current_state,
            "e_y": self.lane_info['e_y'],
            "theta_e": self.lane_info['theta_e'],
            "speed": target_speed,
            "timestamp": time.perf_counter()
        }

        self.controlSender.send(command)

    def thread_work(self):
        """
        Main Behavioral Loop:
        Update Inputs -> Transition States -> Execute Action
        """
        self.update_inputs()
        self.update_state()
        self.execute_behavior()