if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.perception.Perception.threads.threadLane import threadLane
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber

class processPerception(WorkerProcess):
    """This process handles Perception.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

	# Subscribe to StateChange messages to monitor system transitions [cite: 1309]
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, deliveryMode="lastOnly", subscribe=True
        )

        super(processPerception, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        pass

    def process_work(self):
        pass

    def _init_threads(self):
        """Create the Perception Publisher thread and add to the list of threads."""
        LaneTh = threadLane(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(LaneTh)
