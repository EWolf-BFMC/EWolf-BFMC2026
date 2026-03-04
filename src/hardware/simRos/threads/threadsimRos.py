# ==============================================================================
# THREAD FLOW DESCRIPTION: simRos (Simulation Bridge)
# 
# INPUT FROM ROS (GAZEBO):
#   - Topic: /automobile/image_raw
#   - Format: ROS Image message (sensor_msgs/Image)
#   - Purpose: Receive raw video frames from the simulator's virtual camera.
#
# OUTPUT TO ROS (GAZEBO):
#   - Topic: /automobile/command
#   - Format: String (std_msgs/String) -> "speed:val", "steer:val", "brake:val"
#   - Purpose: Publish movement commands to the vehicle model in the simulator.
#
# INPUT FROM BRAIN (INTERNAL): 
#   - Names: SpeedMotor (ID 1), SteerMotor (ID 2), Brake (ID 4) 
#   - Format: String (str)
#   - Source: Dashboard or Control Logic (via Gateway)
#   - Delivery: 'lastOnly' mode to eliminate control lag.
#
# OUTPUT TO BRAIN (INTERNAL):
#   - Name: mainCamera (ID 1) 
#   - Format: String (str) Base64 encoded 
#   - Destination: Dashboard / Vision processes (via Gateway) 
# ==============================================================================


# Import ROS libraries
import rospy
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Import utility libraries
import cv2
import base64

# Import Brain templates and messages
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (serialCamera, SpeedMotor, SteerMotor, Brake, Klem, BatteryLvl)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

class threadsimRos(ThreadWithStop):
    """Bridge thread between Brain internal queues and ROS/Gazebo simulator."""

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.bridge = CvBridge()
        
        # Internal handlers
        self.speedSub = None
        self.steerSub = None
        self.brakeSub = None
        self.cameraSender = None
        self.klSub = None
        self.klFeedback = None
        
        # ROS objects
        self.ros_publisher = None
        
        self.subscribe()
        super(threadsimRos, self).__init__()

    def subscribe(self):
        """Initialize your official Message Handlers for internal communication."""
        # Using messageHandlerSubscriber to listen to movement commands
        self.speedSub = messageHandlerSubscriber(self.queuesList, SpeedMotor, deliveryMode="lastOnly", subscribe=True)
        self.steerSub = messageHandlerSubscriber(self.queuesList, SteerMotor, deliveryMode="lastOnly", subscribe=True)
        self.brakeSub = messageHandlerSubscriber(self.queuesList, Brake, deliveryMode="lastOnly", subscribe=True)
        self.klSub = messageHandlerSubscriber(self.queuesList, Klem, deliveryMode="lastOnly", subscribe=True)

        # Using messageHandlerSender to send camera data back to the Brain
        self.cameraSender = messageHandlerSender(self.queuesList, serialCamera)
        # Fixed: Variable name consistently used as klFeedback
        self.klFeedback = messageHandlerSender(self.queuesList, BatteryLvl)

    def camera_callback(self, data):
        """Triggered every time Gazebo sends a new frame."""
        try:
            # Convert ROS Image to OpenCV matrix
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Encode to Base64 string (as defined in your allMessages.py for mainCamera)
            _, buffer = cv2.imencode('.jpg', cv_image)
            base64_img = base64.b64encode(buffer).decode('utf-8')
            
            # Send the image to the Brain
            self.cameraSender.send(base64_img)
        except Exception as e:
            self.logging.error(f"SimROS Bridge Image Error: {e}")

    def thread_work(self):
        """Main loop: Connects to ROS and monitors internal Brain commands."""
        # Initialize the ROS node for this component
        rospy.init_node('EWolf_SimBridge', anonymous=True, disable_signals=True)
        
        # ROS Subscriber: Listen to the virtual camera
        rospy.Subscriber("/automobile/image_raw", Image, self.camera_callback)
        
        # ROS Publisher: Prepare to send movement commands to Gazebo
        self.ros_publisher = rospy.Publisher("/automobile/command", String, queue_size=1)
        
        # Main loop to check internal Brain queues
        while not self._blocker.is_set():
            # Check for new speed commands from the Brain
            speed_val = self.speedSub.receive()
            if speed_val is not None:
                # Fixed: Use 'speed' key and scaling as per RcBrainThread example
                speed_final = float(speed_val) / 1000.0
                self.ros_publisher.publish(json.dumps({"action": "1", "speed": speed_final}))
            
             # Check for new steering commands from the Brain
            steer_val = self.steerSub.receive()
            if steer_val is not None:
                # Fixed: Use 'steerAngle' key as per RcBrainThread example
                steer_final = float(steer_val)
                self.ros_publisher.publish(json.dumps({"action": "2", "steerAngle": steer_final}))
            
            # Check for braking commands
            brake_val = self.brakeSub.receive()
            if brake_val is not None:
                # Fixed: Use 'steerAngle' key for action 3 as per control.py release example
                self.ros_publisher.publish(json.dumps({"action": "3", "steerAngle": 0.0}))

            # Klem changes
            kl_val = self.klSub.receive()
            if kl_val is not None:
                kl_val_int = int(float(kl_val))
                # Fixed: Use 'activate' key for action 4 as per RcBrainThread example
                self.ros_publisher.publish(json.dumps({"action": "4", "activate": kl_val_int}))
                # Send feedback to the dashboard (BatteryLvl expects int)
                self.klFeedback.send(kl_val_int)
                self.logging.info(f"KL changed to {kl_val_int}")
            
            # Small sleep to prevent high CPU usage in the loop
            rospy.sleep(0.01)
            
        # Clean shutdown for ROS
        rospy.signal_shutdown("Thread stopped")