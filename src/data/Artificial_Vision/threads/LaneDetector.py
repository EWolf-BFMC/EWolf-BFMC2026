from src.templates.threadwithstop import ThreadWithStop 
# Importing the official Enum for communication 
from src.utils.messages.allMessages import mainCamera, HighwayZone, StanleyControl  
import cv2 
import numpy as np
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

class LaneDetector(ThreadWithStop): 
    def __init__(self, queueList, logging, ready_event=None, debugging=False): 
        #Initialize empty thread
        super(LaneDetector, self).__init__()
        self.queuesList = queueList 
        self.logging = logging 
        self.debugging = debugging
        self.ready_event = ready_event

        #Create HANDLERS manually:
        self.messageHandlerSubscriber = messageHandlerSubscriber(self.queuesList, self.logging)
        self.messageHandlerSender = messageHandlerSender(self.queuesList, self.logging)

        #Let main know is ready
        self.subscribe()
        if self.ready_event is not None:
            self.ready_event.set()

    def subscribe(self): 
        """Requests the official camera feed (VAR)""" 
            #lastOnly=True
        super().subscribe(mainCamera)

    def thread_work(self): 
        """Main loop: Look -> Process -> Send data via the StanleyControl channel""" 
        while not self.is_stopped(): 
            image_data = self.messageHandlerSubscriber.get_message(mainCamera) 
            
            if image_data: 
                frame = image_data['value'] 
                
                # 1. Perspective Transform: Bird's Eye View for the 35cm track 
                bev_frame = self.apply_birds_eye(frame) 
                
                # 2. Technical Analysis: Extracting lateral and angle errors 
                lateral_error, heading_error, is_highway = self.calculate_lane_data(bev_frame) 
                
                # 3. THE ASSIST: Packaging data using the NEW StanleyControl Enum 
                control_msg = { 
                    "Type": StanleyControl,  # Official registered message type 
                    "Value": { 
                        "e_y": lateral_error,     # Lateral distance error 
                        "theta_e": heading_error,  # Angle relative to the lane 
                        "speed": 0.5 if is_highway else 0.3 # Turbo mode detection 
                    }, 
                    "Owner": "processController" 
                } 
                
                # Sending packet to the General Queue for the Brain to process 
                self.queuesList["General"].put(control_msg) 

    def apply_birds_eye(self, frame): 
        """Flattens the image to view the 35cm lane without perspective distortion""" 
        h, w = frame.shape[:2] 
        # Coordinates for the Bosch car track 
        src = np.float32([[w*0.3, h*0.7], [w*0.7, h*0.7], [0, h], [w, h]]) 
        dst = np.float32([[0, 0], [w, 0], [0, h], [w, h]]) 
        matrix = cv2.getPerspectiveTransform(src, dst) 
        return cv2.warpPerspective(frame, matrix, (w, h)) 

    def calculate_lane_data(self, bev_frame): 
        """Detects lane markings and calculates the geometry required for Stanley""" 
        hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV) 
        lower_white = np.array([0, 0, 180])  
        upper_white = np.array([180, 40, 255]) 
        mask = cv2.inRange(hsv, lower_white, upper_white) 
        
        edges = cv2.Canny(mask, 50, 150) 
        height, width = edges.shape 
        
        # Hough Transform to identify the white lane lines 
        lines = cv2.HoughLinesP( 
            edges[int(height/2):, :],  
            rho=1, theta=np.pi/180, threshold=30,  
            minLineLength=15, maxLineGap=120 
        ) 
        
        centers, angles, max_line_length = [], [], 0 
        
        if lines is not None: 
            for line in lines: 
                x1, y1, x2, y2 = line[0] 
                centers.append((x1 + x2) / 2) 
                
                # Heading Angle Calculation (Theta_e) 
                angle = np.arctan2(y2 - y1, x2 - x1) 
                angles.append(angle) 
                
                # Highway Detection: Length of the marking segments 
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2) 
                if length > max_line_length: 
                    max_line_length = length 
            
            # 65px threshold for 9cm highway markings 
            is_highway = max_line_length > 65  
            error = np.mean(centers) - (width / 2) 
            heading_err = np.mean(angles) if angles else 0 
            
            return error, heading_err, is_highway 
            
        return 0, 0, False