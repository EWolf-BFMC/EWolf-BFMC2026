# ==============================================================================
# THREAD FLOW DESCRIPTION:
# 
# INPUT: 
#   - Name: serialCamera 
#   - Format: Base64 encoded string representing a low-res image 
#   - Source: threadCamera (via Gateway)
#
# PROCESSING:
#   - Decoding: Base64 string -> NumPy array -> OpenCV BGR Mat 
#   - Geometry: Applies Bird's Eye View and calculates lane errors 
#
# OUTPUT:
#   - Name: StanleyControl 
#   - Format: Dictionary with control parameters 
#   - Destination: threadStanley (via Gateway) 
# ==============================================================================

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import cv2
import numpy as np
import base64
from src.utils.messages.allMessages import serialCamera, StanleyControl


class threadLane(ThreadWithStop):
    """This thread handles Perception.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        # Sender to communicate perception results to the StanleyControl
        self.controlSender = messageHandlerSender(self.queuesList, StanleyControl)
        super(threadLane, self).__init__()

    def subscribe(self):
        # Subscribe to the shrunken camera feed to reduce CPU load
        # 'deliveryMode="lastOnly"' ensures we only process the most recent frame
        self.cameraSubscriber = messageHandlerSubscriber(
            self.queuesList, 
            serialCamera, 
            deliveryMode="lastOnly", 
            subscribe=True
        )


    def state_change_handler(self):
        pass

    def thread_work(self):
        """Main loop: Receive -> Process -> Send data to threadStanley"""
        # Get the message from the camera queue
        image_msg = self.cameraSubscriber.receive()
        
        if image_msg is not None:
            try:
                # 1. Decoding: Data arrives as a Base64 encoded string from Bosch architecture
                img_data = base64.b64decode(image_msg)
                np_img = np.frombuffer(img_data, dtype=np.uint8)
                frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

                if frame is not None:
                    # 2. Perspective Transform (Bird's Eye View)
                    # This prepares the image for technical analysis
                    bev_frame = self.apply_birds_eye(frame)
                    
                    # 3. Technical Analysis: Extract lateral (e_y) and heading (theta_e) errors
                    lateral_error, heading_error, is_highway = self.calculate_lane_data(bev_frame)
                    
                    # 4. Transmission: Package data for threadStanley
                    # These keys (e_y, theta_e) are the standardized input for the Stanley Law
                    control_data = {
                        "e_y": lateral_error,     # Lateral distance error from lane center
                        "theta_e": heading_error,  # Heading angle error relative to the lane
                        "speed": 0.5 if is_highway else 0.3 # Adaptive speed based on zone detection
                    }
                    self.controlSender.send(control_data)
                    
            except Exception as e:
                if self.debugging:
                    self.logging.error(f"Error in threadLane: {e}")

    def apply_birds_eye(self, frame):
        """Flattens the image to view the 35cm lane without perspective distortion."""
        h, w = frame.shape[:2]
        # Coordinates tuned for the shrunken Bosch track view
        src = np.float32([[w*0.2, h*0.7], [w*0.8, h*0.7], [0, h], [w, h]])
        dst = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, matrix, (w, h))

    def calculate_lane_data(self, bev_frame):
        """Detects lane markings and calculates geometry for Stanley algorithm."""
        hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV)
        # Filtering for white lane lines (Bosch track standard)
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        edges = cv2.Canny(mask, 50, 150)
        height, width = edges.shape
        
        # Hough Transform to detect lines in the bottom half of the BEV
        lines = cv2.HoughLinesP(
            edges[int(height/2):, :], 
            rho=1, theta=np.pi/180, threshold=30, 
            minLineLength=15, maxLineGap=120
        )
        
        if lines is not None:
            centers, angles, max_line_length = [], [], 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                centers.append((x1 + x2) / 2)
                # Calculating the heading error (angle theta_e)
                angles.append(np.arctan2(y2 - y1, x2 - x1))
                
                # Check for highway markings (9cm dash segments)
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if length > max_line_length:
                    max_line_length = length
            
            # Threshold adjusted for shrunken image resolution
            is_highway = max_line_length > 40 
            # Calculating lateral error (e_y) as offset from image center
            error = np.mean(centers) - (width / 2)
            heading_err = np.mean(angles) if angles else 0
            return error, heading_err, is_highway
            
        return 0, 0, False

