import numpy as np
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import StanleyControl, SpeedMotor, SteerMotor, State, StateChange
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

class threadLaneFollower(ThreadWithStop):
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        #Initialize empty thread
        super(threadLaneFollower, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.ready_event = ready_event

        # Internal variable to track the system state from the State Machine
        self.current_mode = "Manual"
     
        #Create SUSCRIPTORS per message:
        self.subStanley = messageHandlerSubscriber(queueList, StanleyControl, deliveryMode="lastonly", subscribe=True)
        self.subState = messageHandlerSubscriber(queueList, StateChange, deliveryMode="fifo", subscribe=True)
        
        #Handler to send commands
        self.messageHandlerSender = messageHandlerSender(self.queuesList, self.logging)

        # Stanley Controller Parameters
        self.k = 0.55
        self.ks = 0.1
        self.max_steer = 0.436  # Limits steering to approx +/- 25 degrees

        #Let main know is ready
        #self.subscribe()
        if self.ready_event is not None:
            self.ready_event.set()

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
                time.sleep(0.1)

    def send_commands(self, speed, steer):
        """Publish speed and steering values to motor control threads"""
        speed_int = int(speed * 30)    #Transform speed from float to int
        steer_deg = int(np.rad2deg(steer))  #Transform into rad
        self.messageHandlerSender.send_message(SpeedMotor, str(speed_int))
        self.messageHandlerSender.send_message(SteerMotor, str(steer_deg))

        if self.debugging:
            self.logging.info(f"Status: {self.current_mode} | Vel: {speed_int} | Steer: {steer_deg}")

    def state_change_handler(self):
        """Handle specific transitions if required by the competition architecture"""
        pass