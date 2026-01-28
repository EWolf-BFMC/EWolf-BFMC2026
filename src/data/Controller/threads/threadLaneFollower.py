import numpy as np
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import StanleyControl, SpeedMotor, SteerMotor, State

class threadLaneFollower(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # Stanley Controller Parameters
        self.k = 0.55
        self.ks = 0.1
        self.max_steer = 0.436  # Limits steering to approx +/- 25 degrees

        # Internal variable to track the system state from the State Machine
        self.current_mode = "Manual"

        self.subscribe()
        super(threadLaneFollower, self).__init__()

    def subscribe(self):
        """Subscribe to perception data (vision) and system state"""
        self.messageHandlerSubscriber.subscribe_to_message(StanleyControl)
        self.messageHandlerSubscriber.subscribe_to_message(StateChange)

    def thread_work(self):
        """Main loop with safety filter based on current mode"""
        while not self.is_stopped():
            # 1. UPDATE SYSTEM MODE (Listening to StateMachine/Dashboard)
            state_msg = self.messageHandlerSubscriber.get_message(StateChange)
            if state_msg:
                self.current_mode = state_msg['value']

            # 2. ACTUATION LOGIC: Only execute if mode is 'AUTO'
            if self.current_mode == "AUTO":
                vision_message = self.messageHandlerSubscriber.get_message(StanleyControl)

                if vision_message:
                    data = vision_message['Value']
                    e_y = data['e_y']      # Lateral error
                    theta_e = data['theta_e']  # Heading error
                    v = data['speed']      # Current velocity

                    # Stanley Control Law Calculation:
                    # $$\delta(t) = \theta_e(t) + \arctan\left(\frac{k \cdot e_y(t)}{v(t) + k_s}\right)$$
                    steering_adj = np.arctan2(self.k * e_y, v + self.ks)
                    steering_angle = theta_e + steering_adj

                    # Output saturation to protect steering hardware
                    steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)

                    # Dispatch commands to actuator queues
                    self.send_commands(v, steering_angle)
            else:
                # Standby mode: Do not send commands if mode is not AUTO
                pass

    def send_commands(self, speed, steer):
        """Publish speed and steering values to motor control threads"""
        speed_int = int(speed * 300)    #Transform speed from float to int
        steer_deg = int(np.rad2deg(steer))  #Transform into rad
        self.messageHandlerSender.send_message(SpeedMotor, str(speed_int))
        self.messageHandlerSender.send_message(SteerMotor, str(steer_deg))

        if self.debugging:
            self.logging.info(f"Status: {self.current_mode} | Vel: {speed_int} | Steer: {steer_deg}")

    def state_change_handler(self):
        """Handle specific transitions if required by the competition architecture"""
        pass