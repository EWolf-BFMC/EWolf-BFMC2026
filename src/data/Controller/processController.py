# --- Imports oficiales ---
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
        # Inicializamos el proceso padre
        super(processController, self).__init__(self.queuesList, ready_event)

    def _init_threads(self):
        """
        Hook method to initialize threads within this process.
        The WorkerProcess parent class calls this automatically.
        """
        # Creamos la instancia de tu hilo de control
        LaneFollowerTh = threadLaneFollower(
            self.queuesList, self.logging, self.debugging
        )
        # Lo añadimos a la lista de hilos para que el proceso lo arranque
        self.threads.append(LaneFollowerTh)

    def state_change_handler(self):
        """Handle system-wide state changes if needed (e.g., Emergency Stop)"""
        pass

    def process_work(self):
        """Main process loop (usually empty if logic is inside threads)"""
        pass