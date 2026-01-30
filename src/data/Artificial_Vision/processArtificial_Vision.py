# Location: src/data/Artificial_Vision/processArtificial_Vision.py
from src.templates.workerprocess import WorkerProcess
from src.data.Artificial_Vision.threads.LaneDetector import LaneDetector
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
# We need to import mainCamera so the subscriber knows what to listen to
from src.utils.messages.allMessages import mainCamera, StateChange

class processArtificial_Vision(WorkerProcess):
    """Host process for Computer Vision threads."""
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # --- CORE CORRECTION ---
        # We tune the "radio" to mainCamera so Angel's LaneDetector can receive frames.
        # If Angel also needs to know the system mode, we can use a list: [mainCamera, StateChange]
        self.messageHandlerSubscriber = messageHandlerSubscriber(
            queueList, 
            mainCamera, 
            "lastOnly", 
            True
        )
        
        # We only pass 2 positional arguments to the parent class (WorkerProcess)
        super(processArtificial_Vision, self).__init__(queueList, ready_event)

    def _init_threads(self):
        """Initialize and start the Lane Detection thread."""
        # We pass the pre-configured subscriber instance to the thread
        Artificial_VisionTh = LaneDetector(
            self.messageHandlerSubscriber, 
            self.queuesList, 
            self.logging, 
            self.debugging
        )
        self.threads.append(Artificial_VisionTh)