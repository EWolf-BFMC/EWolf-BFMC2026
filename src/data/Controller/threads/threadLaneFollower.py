# Location: src/data/Controller/threads/threadLaneFollower.py
import numpy as np
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import StanleyControl, SpeedMotor, SteerMotor, StateChange

class threadLaneFollower(ThreadWithStop):
    """
    Autonomous Navigation Controller implementing the Stanley Control Law.
    This thread only actuates when the system mode is set to 'AUTO'.
    Manual control commands bypass this thread and are handled by the Gateway.
    """
    def __init__(self, messageHandlerSubscriber, queueList, logging, debugging=False):
        self.messageHandlerSubscriber = messageHandlerSubscriber
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # --- Stanley Controller Parameters ---
        self.k = 0.55             # Convergence gain
        self.ks = 0.1             # Softening constant
        self.max_steer = 0.436    # Max steering limit (~25 degrees)
        
        # --- Operational State ---
        self.current_mode = "MANUAL" 
        
        self.subscribe()
        super(threadLaneFollower, self).__init__()

    def subscribe(self):
        """
        Starts the subscription for the pre-configured message streams.
        The messages (StanleyControl and StateChange) were already 
        defined in the processController.py file.
        """
        # --- CORRECTION APPLIED HERE ---
        # Just call .subscribe() without arguments. 
        # The subscriber already knows to listen to StanleyControl and StateChange.
        self.messageHandlerSubscriber.subscribe() 

    def thread_work(self):
        """Main Loop: Mode Synchronization -> Stanley Calculation -> Actuation."""
        while not self._is_stopped:
            # 1. SYNC SYSTEM MODE
            state_msg = self.messageHandlerSubscriber.get_message(StateChange)
            if state_msg:
                # Synchronize mode (forcing uppercase for reliability)
                self.current_mode = str(state_msg['value']).upper()

            # 2. AUTONOMOUS GATEKEEPER
            if self.current_mode == "AUTO":
                vision_message = self.messageHandlerSubscriber.get_message(StanleyControl)
                
                if vision_message:
                    # Extract perception data
                    data = vision_message['Value']
                    e_y = data['e_y']         
                    theta_e = data['theta_e'] 
                    v = data['speed']         
                    
                    # 3. STANLEY CONTROL LAW
                    steering_adj = np.arctan2(self.k * e_y, v + self.ks)
                    steering_angle = theta_e + steering_adj
                    
                    # 4. HARDWARE PROTECTION
                    steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
                    
                    # 5. DISPATCH COMMANDS
                    self.send_commands(v, steering_angle)
                    
                    # 6. TELEMETRY / DEBUG MODE 
                    if self.debugging:
                        self.logging.info(
                            f"STANLEY DEBUG | Mode: {self.current_mode} | "
                            f"e_y: {e_y:.2f} | th_e: {theta_e:.2f} | "
                            f"Steer: {np.rad2deg(steering_angle):.1f} deg"
                        )
            else:
                # Standby: Thread waits for 'AUTO' mode notification 
                pass

    def send_commands(self, speed, steer):
        """Publish speed and steering data to the actuators via General Queue."""
        speed_int = int(speed * 300) 
        steer_deg = int(np.rad2deg(steer))
        
        self.queuesList["General"].put({"Type": SpeedMotor, "Value": str(speed_int)})
        self.queuesList["General"].put({"Type": SteerMotor, "Value": str(steer_deg)})