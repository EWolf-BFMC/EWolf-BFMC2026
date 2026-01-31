if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")
    
from src.templates.workerprocess import WorkerProcess
from src.data.Controller.threads.threadLaneFollower import threadLaneFollower

class processController(WorkerProcess):
    """
    This process manages the Controller logic by hosting the LaneFollower thread.
    It inherits from WorkerProcess to handle the process lifecycle and communication.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        # Initialize father process
        super(processController, self).__init__(self.queuesList, ready_event)

    def _init_threads(self):
        """
        Hook method to initialize threads within this process.
        The WorkerProcess parent class calls this automatically.
        """
        # Create Lane Control thread
        LaneFollowerTh = threadLaneFollower(
            self.queuesList, self.logging, self.debugging
        )
        # Add it to the thread list
        self.threads.append(LaneFollowerTh)

    def state_change_handler(self):
        """Handle system-wide state changes if needed (e.g., Emergency Stop)"""
        pass

    def process_work(self):
        """Main process loop (usually empty if logic is inside threads)"""
        pass