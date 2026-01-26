from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import mainCamera, HighwayZone
import cv2
import numpy as np

class LaneDetector(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        super(LaneDetector, self).__init__()

    def subscribe(self):
        """Tells the car to start 'watching' the camera, only the last picture"""
        self.messageHandlerSubscriber.subscribe_to_message(mainCamera, lastOnly=True)

    def thread_work(self):
        """Main loop: Get photo -> Process -> Calculate"""
        while not self.is_stopped():
            image_data = self.messageHandlerSubscriber.get_message(mainCamera)
            
            if image_data:
                frame = image_data['value']
                # 1. Perspective Transform (Bird's Eye View)
                bev_frame = self.apply_birds_eye(frame)
                # 2. Lane and Zone Detection for 35cm lane
                error, is_highway = self.calculate_lane_error(bev_frame)
                
                # 3. Notify the 'Brain' if we are in Highway (lines of 9cm)
                if is_highway:
                    self.notify_highway(True)

    def apply_birds_eye(self, frame):
        """Flattens the image to see the 35cm lane from above"""
        h, w = frame.shape[:2]
        src = np.float32([[w*0.3, h*0.7], [w*0.7, h*0.7], [0, h], [w, h]])
        dst = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, matrix, (w, h))

    def calculate_lane_error(self, bev_frame):
        """Logic for detecting 4.5cm and 9cm white lines"""
        hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 180]) 
        upper_white = np.array([180, 40, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        edges = cv2.Canny(mask, 50, 150)
        height, width = edges.shape
        
        lines = cv2.HoughLinesP(
            edges[int(height/2):, :], 
            rho=1, theta=np.pi/180, threshold=30, 
            minLineLength=15, maxLineGap=120
        )
        
        centers = []
        max_line_length = 0
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                centers.append((x1 + x2) / 2)
                # Measuring the length to find the Highway
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if length > max_line_length:
                    max_line_length = length
            
            # If segments > 65px (~9cm), we are in Highway!
            is_highway = max_line_length > 65 
            lane_center_px = np.mean(centers)
            error = lane_center_px - (width / 2)
            return error, is_highway
            
        return 0, False

    def notify_highway(self, detected):
        """Sends a message to the State Machine to speed up"""
        msg = {'value': detected}
        self.messageHandlerSender.send_message(HighwayZone, msg)