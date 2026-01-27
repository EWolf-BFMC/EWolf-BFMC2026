import numpy as np
from src.templates.threadwithstop import ThreadWithStop
# Importing the official communication IDs
from src.utils.messages.allMessages import StanleyControl, SpeedMotor, SteerMotor

class threadLaneFollower(ThreadWithStop):
    """
    This thread handles the vehicle control using the Stanley Controller algorithm.
    It receives lane geometry data from the Vision thread and sends speed/steering 
    commands to the hardware actuators.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # Stanley Controller Parameters (Tunable)
        self.k = 0.55     # Gain parameter: higher = more aggressive steering
        self.ks = 0.1     # Softening constant to avoid division by zero
        self.max_steer = 0.436  # Max steering angle in radians (approx 25 degrees)
        
        self.subscribe()
        super(threadLaneFollower, self).__init__()

    def subscribe(self):
        """Subscribe to the Vision data stream"""
        self.messageHandlerSubscriber.subscribe_to_message(StanleyControl)

    def thread_work(self):
        """Main control loop: Listen -> Calculate Stanley -> Actuate"""
        while not self.is_stopped():
            # 1. Receive data package from Angel's Vision thread
            message = self.messageHandlerSubscriber.get_message(StanleyControl)
            
            if message:
                data = message['Value']
                e_y = data['e_y']           # Cross-track error (m or px)
                theta_e = data['theta_e']   # Heading error (rad)
                v = data['speed']           # Target speed from Vision
                
                # 2. STANLEY CONTROL LAW CALCULATION
                # Formula: Delta = theta_e + atan( (k * e_y) / (v + ks) )
                steering_adjustment = np.arctan2(self.k * e_y, v + self.ks)
                steering_angle = theta_e + steering_adjustment
                
                # 3. SATURATION (Hardware Protection)
                # Ensure we don't exceed the physical limits of the steering servo
                steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
                
                # 4. EXECUTION
                self.send_commands(v, steering_angle)

    def send_commands(self, speed, steer):
        """Encapsulates the sending of messages to the actuators"""
        # Sending Speed Command
        self.messageHandlerSender.send_message(SpeedMotor, {"value": speed})
        
        # Sending Steering Command (converted to whatever unit your Nucleo expects, usually rad)
        self.messageHandlerSender.send_message(SteerMotor, {"value": steer})
        
        if self.debugging:
            self.logging.info(f"Stanley Control -> Speed: {speed:.2f} | Steer: {steer:.2f}")

    def state_change_handler(self):
        """Handle state changes if the State Machine requires it"""
        pass