if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.control.Control.threads.threadControl import threadControl
from src.control.Control.threads.threadFSM import threadFSM
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber

class processControl(WorkerProcess):
    """This process handles Control.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # Subscribe to StateChange messages to monitor system transitions
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, deliveryMode="lastOnly", subscribe=True
        )

        super(processControl, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value["Control"]["process"]
            if modeDict["enabled"] == True:
                self.resume_threads()
            elif modeDict["enabled"] == False:
                self.pause_threads()


    def process_work(self):
        pass

    def _init_threads(self):
        """Create the Control Publisher thread and add to the list of threads."""
        ControlTh = threadControl(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(ControlTh)
        FsmTh = threadFSM(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(FsmTh)
