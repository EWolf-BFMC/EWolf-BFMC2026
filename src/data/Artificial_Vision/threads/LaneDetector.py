# Location: src/data/Artificial_Vision/threads/LaneDetector.py
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import mainCamera, StanleyControl
import cv2
import numpy as np

class LaneDetector(ThreadWithStop):
    """
    Thread responsible for image processing and Stanley Error extraction.
    It applies perspective transformation and lane detection to calculate
    lateral and heading errors.
    """
    def __init__(self, messageHandlerSubscriber, queueList, logging, debugging=False):
        self.messageHandlerSubscriber = messageHandlerSubscriber
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        super(LaneDetector, self).__init__()

    def subscribe(self):
        """
        Activates the camera feed subscription. 
        Parameters were already configured in processArtificial_Vision.py.
        """
        # --- CORRECTION APPLIED ---
        # No arguments needed here to avoid TypeError.
        self.messageHandlerSubscriber.subscribe()

    def thread_work(self):
        """Main Loop: Capture Frame -> Process BEV -> Calculate Errors -> Dispatch."""
        while not self._is_stopped:
            # Request the latest frame from the camera subscriber
            image_data = self.messageHandlerSubscriber.get_message(mainCamera)
            
            if image_data:
                frame = image_data['value']
                
                # Step 1: Bird's Eye View transformation to flatten the track
                bev_frame = self.apply_birds_eye(frame)
                
                # Step 2: Extract lane geometry and calculate Stanley errors (e_y, theta_e)
                lat_err, head_err, is_hwy = self.calculate_lane_data(bev_frame)
                
                # Step 3: Package data using the StanleyControl message format
                control_msg = {
                    "Type": StanleyControl,
                    "Value": {
                        "e_y": lat_err,      # Lateral error (distance to center)
                        "theta_e": head_err,  # Heading error (orientation)
                        "speed": 0.5 if is_hwy else 0.3 # Target speed based on track type
                    },
                    "Owner": "processController"
                }
                
                # Send the perception data to the General queue for the Controller
                self.queuesList["General"].put(control_msg)

    def apply_birds_eye(self, frame):
        """Applies a perspective transform for 35cm lane width analysis."""
        h, w = frame.shape[:2]
        # Source points: Area of interest on the track
        src = np.float32([[w*0.3, h*0.7], [w*0.7, h*0.7], [0, h], [w, h]])
        # Destination points: Flattened view
        dst = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
        
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, matrix, (w, h))

    def calculate_lane_data(self, bev_frame):
        """Detects lane lines using HSV filtering and Hough Transform."""
        # Step A: Filter white lines in HSV space
        hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 40, 255]))
        edges = cv2.Canny(mask, 50, 150)
        
        # Step B: Find lines using Probabilistic Hough Transform
        lines = cv2.HoughLinesP(
            edges[int(edges.shape[0]/2):, :], 
            1, np.pi/180, 30, 
            minLineLength=15, 
            maxLineGap=120
        )
        
        centers, angles, max_len = [], [], 0
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                centers.append((x1 + x2) / 2)
                angles.append(np.arctan2(y2 - y1, x2 - x1))
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if length > max_len: max_len = length
        
        if centers:
            # Calculate mean lateral error relative to frame center
            lateral_error = np.mean(centers) - (bev_frame.shape[1] / 2)
            heading_error = np.mean(angles)
            is_highway = max_len > 65
            return lateral_error, heading_error, is_highway
            
        return 0, 0, False