import cv2
import numpy as np
from ultralytics import YOLO  # Import the AI
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import SignDetection 
from src.control.Control.threads.allStates import SignType

class threadSigns(ThreadWithStop):
    """
    Sign perception thread for E-Wolf. 
    It provides the 'Sense' data for traffic signs to the FSM.
    """

    def __init__(self, queueList, logging, debugging, shared_container): # FIXED: __init__
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.shared_container = shared_container
        
        # --- VISION MODEL CONFIGURATION ---
        # Lazy-loaded on first frame to avoid power spike during busy startup phase
        self.model = None
        
        self._consecutive = {}   # {"highway_exit": 3, ...}
        self.CONFIRM_FRAMES = 3

        # --- DISTANCE CALIBRATION ---
        self.focal_length = 984.0
        self.real_width_dict = {
            "traffic_light": 200.0, "stop": 200.0, "parking": 200.0, 
            "crosswalk": 200.0, "priority_road": 200.0, "highway": 200.0, 
            "highway_exit": 200.0, "one_way": 200.0, "roundabout": 200.0, 
            "no_entry": 200.0
        }
        
        # --- MAPPING YOLO TO FSM ENUMS ---
        # Translates the AI text into the language understood by the FSM brain
        self.str_to_enum = {
            "traffic_light": SignType.TRAFFIC_LIGHT, 
            "stop": SignType.STOP,
            "parking": SignType.PARKING,
            "crosswalk": SignType.CROSSWALK,
            "priority_road": SignType.PRIORITY,
            "highway": SignType.HIGHWAY_ENTRY,
            "highway_exit": SignType.HIGHWAY_EXIT,
            "one_way": SignType.ONE_WAY,
            "roundabout": SignType.ROUNDABOUT,
            "no_entry": SignType.NO_ENTRY
        }
        
        # Sender to communicate detections to threadFSM
        self.signSender = messageHandlerSender(self.queuesList, SignDetection)
        self.subscribe()

        super(threadSigns, self).__init__(pause=0.2)  # 5Hz — reduce sustained CPU on Pi5

    def subscribe(self):
        """No subscribers needed; data is pulled from shared memory."""
        pass

    def thread_work(self):
        """Main perception loop: Acquisition -> Vision AI -> Transmission."""
        # 1. ACQUISITION: Take the frame from RAM
        frame = self.shared_container.get('frame')
        
        if frame is not None:
            try:
                # Lazy-load model on first frame so startup power spike is avoided
                if self.model is None:
                    self.logging.warning("[Signs] Loading YOLO model...")
                    self.model = YOLO('models/best_ncnn_model', task='detect')
                    self.logging.warning("[Signs] Model loaded. Waiting for system to stabilize...")
                    import time; time.sleep(2.0)
                    self.logging.warning("[Signs] Ready.")

                # 2. PROCESSING:
                # INCREASED to 640 because we retrained the model to see at +80cm
                input_res = 640
                small_frame = cv2.resize(frame, (input_res, input_res))
                
                # Pass the image to our detection function
                detections = self.detect_signs(small_frame)

                if detections:
                    detected_types = {d['type'] for d in detections}
                    for sign_type in list(self._consecutive.keys()):
                        if sign_type not in detected_types:
                            self._consecutive[sign_type] = 0  # reset

                    confirmed = []
                    for det in detections:
                        t = det['type']
                        self._consecutive[t] = self._consecutive.get(t, 0) + 1
                        self.logging.warning(f"[Signs] RAW: {t.name} cnt={self._consecutive[t]} dist={det['distance']:.0f}mm")
                        if self._consecutive[t] >= self.CONFIRM_FRAMES:
                            confirmed.append(det)

                    for det in confirmed:
                        self.signSender.send(det)
                        self.logging.warning(f"[Signs] CONFIRMED: {det['type'].name} a {det['distance']:.1f}mm")
                    

            except Exception as e:
                self.logging.error(f"[threadSigns] Vision processing error: {e}")

    # ==========================================================================
    # IMPLEMENTATION SPACE
    # ==========================================================================

    def detect_signs(self, frame):
        """
        1. Run YOLO inference.
        2. Filter by confidence.
        3. Calculate distance.
        4. Return detections_list of dicts expected by the FSM.
        """
        # Require a minimum confidence of 80%
        results = self.model(frame, conf=0.75, verbose=False)
        detections_list = []

        if len(results[0].boxes) > 0:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]

                # Translate from String to Enum
                sign_enum = self.str_to_enum.get(class_name, None)

                if sign_enum is not None:
                    # Distance calculation
                    w_px = float(box.xywh[0][2])
                    w_real = self.real_width_dict.get(class_name, 200.0)
                    distance_mm = (w_real * self.focal_length) / w_px

                    # Package in the exact format requested by the FSM
                    payload = {
                        "type": sign_enum,
                        "distance": float(distance_mm)
                    }
                    detections_list.append(payload)

        return detections_list

    def state_change_handler(self):
        pass