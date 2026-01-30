# Location: src/data/Controller/processController.py
from src.templates.workerprocess import WorkerProcess
from src.data.Controller.threads.threadLaneFollower import threadLaneFollower
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StanleyControl, StateChange

class processController(WorkerProcess):
    """Host process for navigation control logic."""
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # --- CORE CORRECTION ---
        # We pass a LIST [StanleyControl, StateChange] so the subscriber
        # monitors both perception data and system mode changes.
        self.messageHandlerSubscriber = messageHandlerSubscriber(
            queueList, 
            [StanleyControl, StateChange], # List of messages to "tune" into
            "lastOnly", 
            True
        )
        
        super(processController, self).__init__(queueList, ready_event)

    def _init_threads(self):
        """Spawn the thread that calculates Steering commands."""
        # Passing the pre-configured subscriber instance to the logic thread
        LaneFollowerTh = threadLaneFollower(
            self.messageHandlerSubscriber, 
            self.queuesList, 
            self.logging, 
            self.debugging
        )
        self.threads.append(LaneFollowerTh)