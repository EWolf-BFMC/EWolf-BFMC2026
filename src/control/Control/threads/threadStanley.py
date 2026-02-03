# ==============================================================================
# THREAD FLOW DESCRIPTION:
# 
# INPUT: 
#   - Name: StanleyControl (ID 101)
#   - Format: Dictionary {"e_y": float, "theta_e": float, "speed": float}
#   - Source: threadLane (via Gateway)
#
# PROCESSING:
#   - Actuation: Only runs when thread is RESUMED by processControl (AUTO mode).
#   - Control Law: delta = theta_e + arctan2(k * e_y, v + ks)
#   - Hardware Scaling: Converts Radians to Degrees and Speed to PWM units.
#
# OUTPUT:
#   - Name: SteerMotor (ID 2) and SpeedMotor (ID 1)
#   - Format: String (str) as required by the MessageConverter
#   - Destination: processSerialHandler (via Gateway) -> NUCLEO
# ==============================================================================

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.utils.messages.allMessages import (
    StanleyControl, SpeedMotor, SteerMotor, StateChange
)
import numpy as np
import math

class threadStanley(ThreadWithStop):
    """This thread handles Stanley Control.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # --- Stanley Controller Parameters ---
        self.k = 0.55             # Convergence gain
        self.ks = 0.1             # Softening constant (prevents division by zero)
        self.max_steer = 25.0     # Max steering limit in degrees for Bosch servo
        
        self.subscribe()
        # Actuator Senders
        self.steerSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedSender = messageHandlerSender(self.queuesList, SpeedMotor)
        
        super(threadStanley, self).__init__()

    def subscribe(self):
        """Initializes subscribers for perception data and system state."""
        # Subscribe to StanleyControl messages from threadLane
        self.visionSubscriber = messageHandlerSubscriber(
            self.queuesList, StanleyControl, deliveryMode="lastOnly", subscribe=True
        )

    def state_change_handler(self):
        pass

    def thread_work(self):
        """Main Loop: Receive Perception -> Calculate Law -> Actuate."""
        # Receive the latest dictionary from threadLane
        vision_message = self.visionSubscriber.receive()
        
        if vision_message:
            try:
                # 1. Extract perception data
                e_y = vision_message.get('e_y', 0)         
                theta_e = vision_message.get('theta_e', 0) 
                v = vision_message.get('speed', 0.3)         
                
                # 2. STANLEY CONTROL LAW (delta = theta_e + arctan(k*e_y / v + ks))
                steering_adj = math.atan2(self.k * e_y, v + self.ks)
                steering_angle_rad = theta_e + steering_adj
                
                # 3. CONVERSION TO HARDWARE UNITS
                # Convert Radians to Degrees for the NUCLEO servo handler
                steering_angle_deg = math.degrees(steering_angle_rad)
                steering_angle_deg = np.clip(steering_angle_deg, -self.max_steer, self.max_steer)
                
                # 4. DISPATCH COMMANDS
                self.send_commands(v, steering_angle_deg)
                
                if self.debugging:
                    self.logging.info(f"STANLEY | e_y: {e_y:.2f} | Steer: {steering_angle_deg:.1f} deg")
            
            except Exception as e:
                self.logging.error(f"Error in threadStanley logic: {e}")

    def send_commands(self, speed, steer_deg):
        """Publishes commands to the serial handler using the General Queue[cite: 111, 746]."""
        # Convert speed to PWM value (0-1000 range typical for Nucleo)
        # Example: 0.3 speed * 300 = 90 units
        speed_val = str(int(speed * 300))
        steer_val = str(float(steer_deg))
        
        self.speedSender.send(speed_val) # Sent as ID 1 
        self.steerSender.send(steer_val) # Sent as ID 2