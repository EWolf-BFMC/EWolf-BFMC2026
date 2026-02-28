# ==============================================================================
# THREAD FLOW DESCRIPTION:
# THIS THREAD CALCULATES LATERAL AND HEADING ERRORS FOR THE CONTROLLER.
# 
# INPUT: 
#   - Name: shared_container['frame']
#   - Format: Raw OpenCV BGR Mat (Zero-copy from RAM)
#   - Source: threadCamera (Internal Process Memory)
#
# PROCESSING:
#   - Bird's Eye View: Perspective transform focused on the 35cm track width.
#   - Temporal Filtering: Moving average (size 5) to eliminate steering jitter.
#   - Geometry: Calculates e_y (lateral) and theta_e (heading) relative to center.
#   - Reliability: Provides a score (0.0 to 1.0) based on detection stability.
#
# OUTPUT:
#   - Name: LaneData
#   - Format: Dictionary {"e_y": float, "theta_e": float, "reliability": float}
#   - Destination: threadLogic (via Gateway) 
# ==============================================================================

import cv2
import numpy as np
import time
from collections import deque
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import LaneData 

class threadLane(ThreadWithStop):
    """
    Lane perception thread for E-Wolf. 
    Validated for Raspberry Pi 5 embedded constraints.
    """

    def __init__(self, queueList, logging, debugging, shared_container):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.shared_container = shared_container
        
        # --- MOVING AVERAGE FILTERS ---
        # Automatic window management using deque
        self.buffer_size = 5      
        self.e_y_buffer = deque(maxlen=self.buffer_size)      
        self.theta_e_buffer = deque(maxlen=self.buffer_size)  
        
        # --- PRE-DEFINED MASK RANGES ---
        self.lower_white = np.array([0, 0, 180])
        self.upper_white = np.array([180, 50, 255])
        
        # --- ENFORCED GEOMETRY ---
        self.EXPECTED_W, self.EXPECTED_H = 512, 270
        # Calibration: BEV dst width (512 px) covers the 35 cm track width.
        # Adjust TRACK_WIDTH_M if the camera FOV or track dimensions differ.
        TRACK_WIDTH_M = 0.35
        self.BEV_PIXELS_PER_METER = self.EXPECTED_W / TRACK_WIDTH_M  # ≈ 1462.9 px/m
        
        src = np.float32([
            [self.EXPECTED_W*0.35, self.EXPECTED_H*0.65], 
            [self.EXPECTED_W*0.65, self.EXPECTED_H*0.65], 
            [self.EXPECTED_W*0.05, self.EXPECTED_H], 
            [self.EXPECTED_W*0.95, self.EXPECTED_H]
        ])
        dst = np.float32([
            [0, 0], [self.EXPECTED_W, 0], 
            [0, self.EXPECTED_H], [self.EXPECTED_W, self.EXPECTED_H]
        ])
        
        self.perspective_matrix = cv2.getPerspectiveTransform(src, dst)
        self.controlSender = messageHandlerSender(self.queuesList, LaneData)
        
        super(threadLane, self).__init__(pause=0.001)

    def thread_work(self):
        """Main perception loop with explicit failure safety."""
        start_time = time.perf_counter()
        frame = self.shared_container.get('frame')
        
        if frame is not None:
            try:
                # 1. GEOMETRY GUARD: Explicit failure > Silent abort
                h, w = frame.shape[:2]
                if w != self.EXPECTED_W or h != self.EXPECTED_H:
                    self.logging.error(f"[Lane] CRITICAL: Resolution mismatch ({w}x{h})")
                    self.controlSender.send({"e_y": 0.0, "theta_e": 0.0, "reliability": 0.0})
                    return

                # 2. TRANSFORM: Warp to Bird's Eye View
                bev_frame = cv2.warpPerspective(frame, self.perspective_matrix, (w, h))
                
                # 3. PERCEPTION: Extract filtered data and reliability
                lat_err, head_err, reliability = self.calculate_filtered_data(bev_frame)
                
                # 4. OUTPUT: Send data to FSM
                self.controlSender.send({
                    "e_y": lat_err,
                    "theta_e": head_err,
                    "reliability": reliability
                })
                
                # 5. DATA LOGGING: Real-time performance monitoring
                loop_time_ms = (time.perf_counter() - start_time) * 1000
                if self.debugging:
                    print(f"[Lane] Loop: {loop_time_ms:.2f}ms | Reliability: {reliability:.2f} | e_y: {lat_err:.2f}")
                
            except Exception as e:
                self.logging.error(f"[threadLane] Processing error: {e}")
                self.controlSender.send({"e_y": 0.0, "theta_e": 0.0, "reliability": 0.0})

    def calculate_filtered_data(self, bev_frame):
        """Detects lane markings with optimized ROI height."""
        h, w = bev_frame.shape[:2]
        hsv = cv2.cvtColor(bev_frame, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        edges = cv2.Canny(mask, 50, 150)
        
        # --- ROI OPTIMIZATION: Ignore top 60% for speed
        roi_edges = edges[int(h*0.6):, :]
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 35, minLineLength=20, maxLineGap=100)
        
        if lines is not None:
            centers, angles = [], []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                centers.append((x1 + x2) / 2)
                angles.append(np.arctan2(y2 - y1, x2 - x1))

            e_y_pixels = np.mean(centers) - (w / 2)
            self.e_y_buffer.append(e_y_pixels / self.BEV_PIXELS_PER_METER)  # convert to metres
            self.theta_e_buffer.append(np.mean(angles) + (np.pi/2))
        
        elif len(self.e_y_buffer) > 0:
            # Drain buffer to alert FSM of lane loss
            self.e_y_buffer.popleft()
            self.theta_e_buffer.popleft()
        
        reliability = len(self.e_y_buffer) / self.buffer_size

        if not self.e_y_buffer:
            return 0.0, 0.0, 0.0
            
        return np.mean(self.e_y_buffer), np.mean(self.theta_e_buffer), reliability