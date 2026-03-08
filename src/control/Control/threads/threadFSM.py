from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.control.Control.threads.allStates import BehaviorState, SignType, ObstacleZone, SpeedLimit
from src.utils.messages.allMessages import (
    ControlAction, FsmStatus, LaneData, LidarObstacle, SignDetection
)
import time

# =============================================================================
# INTERSECTION OPEN-LOOP SEQUENCE
#
# 6 pre-calibrated maneuvers for the planned race route.
# Each entry: (duration_s, speed_m/s, steer_deg, label)
# Executed in order on each successive INTERSECTION state entry.
# Wraps back to 0 after all 6 are consumed.
# =============================================================================
_INTERSECTION_SEQUENCE = [
    (4.0, 0.20,  12.0, "intersection-1"),
    (6.0, 0.20, -15.0, "intersection-2"),
    (2.0, 0.10, -15.0, "intersection-3"),
    (3.0, 0.20,  10.0, "intersection-4"),
    (7.0, 0.20,  -4.0, "intersection-5"),
    (4.0, 0.20,  12.0, "intersection-6"),
]

_DECEL_RAMP_DURATION = 2.0   # DECELERATION RAMP CONSTANT

# =============================================================================
# PARKING MANEUVER CONSTANTS
#
# Sequence: (duration_s, speed_m/s, steer_deg, label)
# All timings are initial estimates — calibrate on the physical car.
# =============================================================================
_PARKING_PHASES = [
    # --- Park IN ---
    (8.0,  -0.10, -23.0, "in-reverse-left"),
    (2.0,  -0.10,  23.0, "in-reverse-right"),
    (2.0,   0.10, -23.0, "in-forward-left"),
    (2.4,  -0.10,  23.0, "in-reverse-right-2"),
    (1.2,   0.10, -14.0, "in-exit-straight"),
    # --- Wait in spot ---
    (3.0,   0.00,   0.0, "wait-in-spot"),
    # --- Park OUT (reverse order, negated speed, same steer) ---
    (1.3,  -0.10,  14.0, "out-reverse-straight"),
    (2.4,   0.10,  23.0, "out-forward-right-2"),
    (2.0,  -0.10, -23.0, "out-reverse-left"),
    (0.75,  0.10,  23.0, "out-forward-right"),
    (3.5,   0.10, -23.0, "out-forward-left"),
]


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
        #PARKING_MANEUVER
        self.current_state = BehaviorState.IDLE  # TEST — revert to IDLE after
        self.previous_state = BehaviorState.IDLE

        # --- INPUT MEMORY ---
        self.lane_info = {"e_y": 0.0, "theta_e": 0.0, "reliability": 0.0}
        self.obstacle_info = {"distance": 9999.0, "reliability": 1.0}
        self.lidar_data_received = False  # True after first real LidarObstacle message
        self.active_sign = {"type": None, "distance": 2000.0}
        self.current_target_speed = SpeedLimit.CITY_MIN.value
        self.stop_timer_start = None
        self.stop_reason = None  # "SIGN" or "PEDESTRIAN"

        # --- MANEUVER STATE TRACKING ---
        # Set intersection_direction externally by a route planner before the
        # car reaches the intersection: "LEFT" | "RIGHT" | "STRAIGHT"
        # TODO: wire to route planner / mission control message.
        self.intersection_direction = "STRAIGHT"
        self.maneuver_complete = False

        self._intersection_phase = 0
        self._intersection_phase_timer = None
        self._intersection_index = 0        # which of the 6 maneuvers to run next
        self._current_intersection = _INTERSECTION_SEQUENCE[0]

        self._parking_phase = 0
        self._parking_phase_timer = None

        self._decel_start_time = None
        self._decel_entry_speed = SpeedLimit.CITY_MIN.value  # updated by the state before DECEL

        # Tracks previous state in execute_behavior() to detect fresh entries
        self._prev_executed_state = None

        self.subscribe()

        # Action Sender (The output of the FSM)
        self.controlSender = messageHandlerSender(self.queuesList, ControlAction)
        self.fsmStatusSender = messageHandlerSender(self.queuesList, FsmStatus)
        self._last_status = None

        super(threadFSM, self).__init__(pause=0.01)  # 100 Hz Decision Loop

    # =========================================================================
    # SENSOR PIPELINE
    # =========================================================================

    def evaluate_obstacle_zone(self):
        """Maps raw distance to a logical zone."""
        # Before the first real lidar packet is received, treat as clear so
        # the FSM doesn't lock in EMERGENCY_BRAKE during sensor startup.
        if not self.lidar_data_received:
            return ObstacleZone.CLEAR

        distance = self.obstacle_info["distance"]
        reliability = self.obstacle_info["reliability"]

        if reliability < 0.2:
            return ObstacleZone.DANGER  # Sensor lost mid-run → emergency stop

        if distance < 300.0:
            return ObstacleZone.DANGER
        elif distance < 900.0:
            return ObstacleZone.WARNING
        else:
            return ObstacleZone.CLEAR

    def subscribe(self):
        """Initializes subscribers for all 'Sense' threads."""
        self.laneSub = messageHandlerSubscriber(
            self.queuesList, LaneData, "lastOnly", True)
        self.lidarSub = messageHandlerSubscriber(
            self.queuesList, LidarObstacle, "lastOnly", True)
        self.signSub = messageHandlerSubscriber(
            self.queuesList, SignDetection, "lastOnly", True)

    def update_inputs(self):
        """
        Polls all subscribers and updates internal memory.
        This represents the 'Sensing' phase of the FSM.
        """
        lane_data = self.laneSub.receive()
        if lane_data:
            self.lane_info['reliability'] = lane_data.get('reliability', 0.0)
            if self.lane_info['reliability'] >= 0.3:
                self.lane_info['e_y']     = lane_data.get('e_y', 0.0)
                self.lane_info['theta_e'] = lane_data.get('theta_e', 0.0)
            else:
                # Lane lost or unreliable — drive straight rather than
                # chasing a stale cross-track error.
                self.lane_info['e_y']     = 0.0
                self.lane_info['theta_e'] = 0.0

        lidar_data = self.lidarSub.receive()
        if lidar_data:
            self.obstacle_info['distance'] = lidar_data.get('distance', 9999.0)
            self.obstacle_info['reliability'] = lidar_data.get('reliability', 0.0)
            self.lidar_data_received = True

        sign_data = self.signSub.receive()
        if sign_data:
            raw_type = sign_data.get('type', None)
            if isinstance(raw_type, SignType):
                self.active_sign['type'] = raw_type
            elif raw_type is not None:
                try:
                    self.active_sign['type'] = SignType(raw_type)
                except ValueError:
                    self.active_sign['type'] = None
            else:
                self.active_sign['type'] = None
            self.active_sign['distance'] = sign_data.get('distance', 2000.0)
        else:
            self.active_sign = {"type": None, "distance": 2000.0}

    # =========================================================================
    # STATE MACHINE
    # =========================================================================

    def update_state(self):
        """Thinking Phase: Transition logic following FSM."""
        zone = self.evaluate_obstacle_zone()
        sign = self.active_sign['type']
        s_dist = self.active_sign['distance']

        # --- PRIORITY 1: EMERGENCY BRAKE ---
        if zone == ObstacleZone.DANGER:
            if self.current_state != BehaviorState.EMERGENCY_BRAKE:
                self.previous_state = self.current_state
                self.current_state = BehaviorState.EMERGENCY_BRAKE
                self.stop_timer_start = None
            return

        if self.current_state == BehaviorState.EMERGENCY_BRAKE:
            if zone == ObstacleZone.CLEAR:
                self.current_state = self.previous_state
                # If we were mid-maneuver, skip remaining phases.
                # The reliability handshake in update_state() will gate
                # the transition back to LANE_FOLLOWING safely.
                if self.previous_state in (BehaviorState.INTERSECTION,
                                           BehaviorState.PARKING_MANEUVER):
                    self.maneuver_complete = True
            elif zone == ObstacleZone.WARNING:
                # Obstacle backed off from DANGER to WARNING — slow approach
                # instead of staying fully stopped.
                self.current_state = BehaviorState.DECELERATING
            return

        # --- PRIORITY 2: TRAFFIC RULES ---

        # IDLE → LANE_FOLLOWING (first cycle after boot)
        if self.current_state == BehaviorState.IDLE:
            self.current_state = BehaviorState.LANE_FOLLOWING

        # LANE_FOLLOWING transitions
        elif self.current_state == BehaviorState.LANE_FOLLOWING:
            if (sign in (SignType.STOP, SignType.PARKING, SignType.CROSSWALK)
                    and s_dist < 600) or (zone == ObstacleZone.WARNING):
                self.current_state = BehaviorState.DECELERATING
            elif sign == SignType.PRIORITY and s_dist < 600:
                self.current_state = BehaviorState.INTERSECTION
            elif sign == SignType.HIGHWAY_ENTRY:
                self.current_state = BehaviorState.HIGHWAY_DRIVING

        # HIGHWAY_DRIVING transitions
        elif self.current_state == BehaviorState.HIGHWAY_DRIVING:
            if sign == SignType.HIGHWAY_EXIT:
                self.current_state = BehaviorState.LANE_FOLLOWING

        # DECELERATING transitions
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

        # STOP_ACTION: 3-second regulatory halt
        elif self.current_state == BehaviorState.STOP_ACTION:
            if self.stop_timer_start is None:
                self.stop_timer_start = time.perf_counter()

            if time.perf_counter() - self.stop_timer_start >= 3.0:
                self.stop_timer_start = None
                if self.stop_reason == "SIGN":
                    self.current_state = BehaviorState.INTERSECTION
                else:  # PEDESTRIAN
                    self.current_state = BehaviorState.LANE_FOLLOWING
                self.stop_reason = None

        # INTERSECTION / PARKING
        # The FSM refuses to leave these states until the open-loop maneuver
        # is fully complete AND threadLane reports stable lane lines (≥ 0.8).
        elif self.current_state in (BehaviorState.INTERSECTION,
                                    BehaviorState.PARKING_MANEUVER):
            if self.maneuver_complete and self.lane_info['reliability'] > 0.8:
                self.current_state = BehaviorState.LANE_FOLLOWING
                self.maneuver_complete = False

    # =========================================================================
    # ACTION DISPATCH
    # =========================================================================

    def _send_command(self, behavior, speed,
                      e_y=0.0, theta_e=0.0, override_steer=None):
        """
        Builds and dispatches a ControlAction packet to threadControl.

        Args:
            behavior (BehaviorState): Current behavioral context.
            speed (float): Target speed in m/s.
            e_y (float): Cross-track error in metres (Stanley input).
            theta_e (float): Heading error in radians (Stanley input).
            override_steer (float | None): If provided, threadControl will
                bypass the Stanley controller and send this fixed steering
                angle (degrees) directly to the NUCLEO.
        """
        command = {
            "behavior": behavior,
            "e_y": e_y,
            "theta_e": theta_e,
            "speed": speed,
            "timestamp": time.perf_counter(),
        }
        if override_steer is not None:
            command["override_steer"] = override_steer
        self.controlSender.send(command)

    def execute_behavior(self):
        """
        Action Phase: Sends the appropriate ControlAction for current_state.
        Detects state transitions (fresh_entry) to initialise open-loop phases.
        """
        state = self.current_state
        fresh_entry = (state != self._prev_executed_state)
        self._prev_executed_state = state

        if state == BehaviorState.IDLE:
            self._action_idle(fresh_entry)
        elif state == BehaviorState.STOP_ACTION:
            self._action_stop()
        elif state == BehaviorState.EMERGENCY_BRAKE:
            self._action_emergency_brake()
        elif state == BehaviorState.LANE_FOLLOWING:
            self._action_lane_following()
        elif state == BehaviorState.HIGHWAY_DRIVING:
            self._action_highway_driving()
        elif state == BehaviorState.DECELERATING:
            self._action_decelerating(fresh_entry)
        elif state == BehaviorState.INTERSECTION:
            self._action_intersection(fresh_entry)
        elif state == BehaviorState.PARKING_MANEUVER:
            self._action_parking(fresh_entry)
        else:
            self._send_command(state, 0.0, override_steer=0.0)

    # =========================================================================
    # PASSIVE / SAFETY ACTIONS
    # =========================================================================

    def _action_idle(self, fresh_entry):
        """
        Reset all internal memory and center steering (0°).

        IDLE lasts exactly one cycle; update_state() transitions to
        LANE_FOLLOWING immediately. The reset here ensures any stale state
        from a previous run cannot leak into the new autonomous session.
        Steering is explicitly centered via override_steer so that
        threadControl also resets its derivative memory.
        """
        if fresh_entry:
            self.lane_info = {"e_y": 0.0, "theta_e": 0.0, "reliability": 0.0}
            self.obstacle_info = {"distance": 2000.0, "reliability": 0.0}
            self.active_sign = {"type": None, "distance": 2000.0}
            self.stop_timer_start = None
            self.stop_reason = None
            self._intersection_phase = 0
            self._intersection_phase_timer = None
            self._parking_phase = 0
            self._parking_phase_timer = None
            self._decel_start_time = None
            self.maneuver_complete = False
            if self.debugging:
                self.logging.info("[FSM] IDLE — full memory reset, steering centred.")

        # override_steer=0.0 → execute_open_loop resets prev_steering_angle_rad
        self._send_command(BehaviorState.IDLE, 0.0, override_steer=0.0)

    def _action_stop(self):
        """
        Hard halt: zero velocity, centered steering, every cycle.

        The 3-second timer is managed by update_state().
        Sending override_steer=0.0 ensures threadControl resets its
        derivative memory so the wheels do not snap on resumption.
        """
        self._send_command(BehaviorState.STOP_ACTION, 0.0, override_steer=0.0)

    def _action_emergency_brake(self):
        """
        Immediate stop, bypassing all control math.

        threadControl gives EMERGENCY_BRAKE the highest priority: it resets
        prev_steering_angle_rad and sends 0,0 before inspecting override_steer.
        previous_state was already saved by update_state() for recovery.
        """
        self._send_command(BehaviorState.EMERGENCY_BRAKE, 0.0, override_steer=0.0)

    # =========================================================================
    # BASE NAVIGATION ACTIONS  (Stanley control)
    # =========================================================================

    def _action_lane_following(self):
        """City navigation at CITY_MIN (20 cm/s); lane errors fed to Stanley."""
        self._decel_entry_speed = SpeedLimit.CITY_MIN.value
        self._send_command(
            BehaviorState.LANE_FOLLOWING,
            SpeedLimit.CITY_MIN.value,          # 0.20 m/s
            e_y=self.lane_info['e_y'],
            theta_e=self.lane_info['theta_e'],
        )

    def _action_highway_driving(self):
        """
        Highway navigation at HIGHWAY_MIN (40 cm/s).
        Right-hand lane discipline is enforced upstream by threadLane's
        ROI; the FSM only raises the target speed.
        """
        self._decel_entry_speed = SpeedLimit.HIGHWAY_MIN.value
        self._send_command(
            BehaviorState.HIGHWAY_DRIVING,
            SpeedLimit.HIGHWAY_MIN.value,       # 0.40 m/s
            e_y=self.lane_info['e_y'],
            theta_e=self.lane_info['theta_e'],
        )

    def _action_decelerating(self, fresh_entry):
        """
        Linear ramp over
        _DECEL_RAMP_DURATION seconds. Stanley tracking is maintained.
        """
        if fresh_entry:
            self._decel_start_time = time.perf_counter()

        elapsed = time.perf_counter() - self._decel_start_time
        t = min(elapsed / _DECEL_RAMP_DURATION, 1.0)
        start  = self._decel_entry_speed        # 0.20 from city, 0.40 from highway
        target = self._decel_entry_speed * 0.5  # ramp to half
        speed  = start + t * (target - start)

        self._send_command(
            BehaviorState.DECELERATING,
            speed,
            e_y=self.lane_info['e_y'],
            theta_e=self.lane_info['theta_e'],
        )

    # =========================================================================
    # MANEUVERING ACTIONS  (Open-loop / timed sequences)
    # =========================================================================

    def _action_intersection(self, fresh_entry):
        """
        Executes the next maneuver in _INTERSECTION_SEQUENCE.

        On each fresh entry the index advances so successive intersections
        run different pre-calibrated open-loop commands.

        Phases
        ------
        0  Maneuver execution — timed speed+steer from _INTERSECTION_SEQUENCE
        1  Slow creep         — 0.05 m/s, steer 0° while awaiting lane reliability
        """
        if fresh_entry:
            self._intersection_phase = 0
            self._intersection_phase_timer = None
            self.maneuver_complete = False
            self._current_intersection = _INTERSECTION_SEQUENCE[self._intersection_index]
            dur, spd, steer, label = self._current_intersection
            self.logging.warning(
                f"[FSM] INTERSECTION #{self._intersection_index + 1} ({label}): "
                f"{dur}s @ {spd*100:.0f}cm/s, steer={steer}°")

        now = time.perf_counter()
        dur, spd, steer, label = self._current_intersection

        # ── Phase 0: Execute pre-calibrated maneuver ──────────────────────────
        if self._intersection_phase == 0:
            if self._intersection_phase_timer is None:
                self._intersection_phase_timer = now
            if now - self._intersection_phase_timer >= dur:
                self._intersection_phase = 1
                self.maneuver_complete = True
                self._intersection_index = (self._intersection_index + 1) % len(_INTERSECTION_SEQUENCE)
                self.logging.warning(
                    f"[FSM] INTERSECTION maneuver done. Next index: {self._intersection_index + 1}")
            self._send_command(BehaviorState.INTERSECTION, spd, override_steer=steer)

        # ── Phase 1: Slow creep — await lane reliability handshake ───────────
        else:
            self._send_command(BehaviorState.INTERSECTION, 0.05, override_steer=0.0)

    def _action_parking(self, fresh_entry):
        """
        Timed open-loop parking sequence (draft — calibrate on the car).

        Phase sequence
        --------------
        0  Approach and align      1.5 s   fwd  0.10 m/s   steer  0°
        1  Clear spot entry        1.0 s   fwd  0.10 m/s   steer +20°
        2  Reverse into spot       2.0 s   rev -0.10 m/s   steer +25°
        3  Straighten in spot      0.5 s   rev -0.08 m/s   steer -15°
        4  Wait in spot            3.0 s   0    m/s         steer  0°
        5  Exit forward            2.0 s   fwd  0.10 m/s   steer -25°
        6  Return to lane          0.8 s   fwd  0.10 m/s   steer +10°
        7  Slow creep              50 mm/s, steer 0° while awaiting reliability

        After phase 6, maneuver_complete=True. update_state() will not exit
        PARKING_MANEUVER until maneuver_complete AND lane reliability ≥ 0.8.
        """
        if fresh_entry:
            self._parking_phase = 0
            self._parking_phase_timer = None
            self.maneuver_complete = False
            if self.debugging:
                self.logging.info("[FSM] PARKING_MANEUVER entered.")

        now = time.perf_counter()
        phase = self._parking_phase

        if phase < len(_PARKING_PHASES):
            duration, speed, steer, _ = _PARKING_PHASES[phase]

            # Initialise timer on the first cycle of each phase
            if self._parking_phase_timer is None:
                self._parking_phase_timer = now

            if now - self._parking_phase_timer >= duration:
                self._parking_phase += 1
                self._parking_phase_timer = now
                if self._parking_phase >= len(_PARKING_PHASES):
                    self.maneuver_complete = True
                    if self.debugging:
                        self.logging.info(
                            "[FSM] PARKING_MANEUVER complete. "
                            "Awaiting lane reliability ≥ 0.8 …")

            self._send_command(BehaviorState.PARKING_MANEUVER,
                               speed, override_steer=steer)

        else:
            # Slow creep while waiting for the reliability handshake
            self._send_command(BehaviorState.PARKING_MANEUVER,
                               0.05,   # slow creep while awaiting lane reliability
                               override_steer=0.0)

    # =========================================================================
    # MAIN LOOP
    # =========================================================================

    def _publish_status(self):
        """Publishes FSM telemetry to the dashboard. Only emits on change."""
        zone = self.evaluate_obstacle_zone()
        sign_type = self.active_sign.get("type")
        status = {
            "state": self.current_state.name,
            "sign": sign_type.name if sign_type is not None else "NONE",
            "obstacle_zone": zone.name,
        }
        if status != self._last_status:
            self._last_status = status
            self.fsmStatusSender.send(status)

    def thread_work(self):
        """
        Main Behavioral Loop:
        Update Inputs → Transition States → Execute Action
        """
        self.update_inputs()
        self.update_state()
        self.execute_behavior()
        self._publish_status()

