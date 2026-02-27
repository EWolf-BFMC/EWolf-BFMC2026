# ==============================================================================
# THREAD FLOW DESCRIPTION:
# THIS THREAD DETECTS AND CLASSIFIES ROAD SIGNS USING COMPUTER VISION.
# 
# INPUT: 
#   - Name: shared_container['frame']
#   - Format: Raw OpenCV BGR Mat (Zero-copy from RAM)
#   - Source: threadCamera (Internal Process Memory)
#
# PROCESSING (NOT DONE YET):
#   - Detection: AI Model (YOLO/TFLite) to locate signs in the frame.
#   - Classification: Mapping detections to BFMC SignType IDs.
#   - Distance: Estimating distance (mm) based on bounding box width.
#
# OUTPUT:
#   - Name: SignDetection
#   - Format: Dictionary {"type": int, "distance": float}
#   - Destination: threadFSM (The Brain)
# ==============================================================================

import cv2
import numpy as np
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import SignDetection 
from src.control.Control.threads.allStates import SignType

class threadSigns(ThreadWithStop):
    """
    Sign perception thread for E-Wolf. 
    It provides the 'Sense' data for traffic signs to the FSM.
    """

    def __init__(self, queueList, logging, debugging, shared_container):
        """
        Args:
            queueList (dict): Dictionary of multiprocessing queues.
            logging (object): Logger for event tracking.
            debugging (bool): If True, enables console feedback.
            shared_container (dict): Shared dictionary for raw frames.
        """
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.shared_container = shared_container
        
        # --- VISION MODEL CONFIGURATION ---
        # SHOULD INITIALIZE the YOLO/TFLite model here
        self.model = None
        
        # Sender to communicate detections to the threadFSM
        self.signSender = messageHandlerSender(self.queuesList, SignDetection)
        self.subscribe()

        super(threadSigns, self).__init__(pause=0.01) # 100Hz loop

    def subscribe(self):
        """No subscribers needed; data is pulled from shared memory."""
        pass

    def thread_work(self):
        """Main perception loop: Acquisition -> Vision AI -> Transmission."""
        # 1. ACQUISITION: Grab the raw BGR frame directly from RAM
        frame = self.shared_container.get('frame')
        
        if frame is not None:
            try:
                # 2. PROCESSING: IMPLEMENT HERE VISION ALGORITHM
                # This is where your will call the model

                #Resizing frame
                # Use 320 or 416 according on how fast is the yolo model
                input_res = 320 
                small_frame = cv2.resize(frame, (input_res, input_res))
                
                # We use the small frame for the AI
                detections = self.detect_signs(small_frame)

                ###############################################33
                
                if detections:
                    for det in detections:
                        # 3. OUTPUT: Send filtered data to the FSM
                        # Expected format: {"type": int, "distance": float}
                        self.signSender.send(det)
                        
                        if self.debugging:
                            print(f"[Signs] Detected ID: {det['type']} at {det['distance']:.1f}mm")
                
            except Exception as e:
                self.logging.error(f"[threadSigns] Vision processing error: {e}")

    # ==========================================================================
    # IMPLEMENTATION SPACE
    # ==========================================================================

    def detect_signs(self, frame):
        """
        IMPLEMENT HERE SIGN DETECTION AND DISTANCE ESTIMATION.
        
        This function should:
        1. Run inference on the frame.
        2. Filter detections by confidence.
        3. Calculate distance, could be using: d = (RealWidth * FocalLength) / PixelWidth
        4. Return a list of dictionaries.
        
        Example Return:
            return [{"type": SignType.STOP.value, "distance": 450.2}]
        """
        # --- ADD CODE HERE ---
        
        return [] # Placeholder

    def state_change_handler(self):
        """Standard handler for system mode changes."""
        pass