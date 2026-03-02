# ==============================================================================
# PROCESS DESCRIPTION:
# THIS PROCESS MANAGES THE LIDAR SENSOR AND OBSTACLE DETECTION LOGIC
# 
# THREADS:
#   - threadReader: Constant acquisition of raw scans from hardware.
#   - threadDetector: Analysis of scans to find frontal obstacles.
#
# SHARED RESOURCES:
#   - shared_container: Dictionary {'last_scan': []} for zero-latency data transfer.
#   - lidar_obj: The hardware instance shared between threads.
# ==============================================================================

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from rplidar import RPLidar
from src.templates.workerprocess import WorkerProcess
from src.hardware.Lidar.threads.threadReader import threadReader
from src.hardware.Lidar.threads.threadDetector import threadDetector

class processLidar(WorkerProcess):
    """This process handles Lidar.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # Initialize shared memory for inter-thread communication
        self.shared_container = {'last_scan': []}
        
        # Hardware initialization: RPLidar A2M8
        try:
            # Using USB0 as verified in your previous tests
            self.lidar_obj = RPLidar('/dev/ttyUSB0')
            self.lidar_obj.clean_input()
            self.lidar_obj.start_motor()
            self.logging.info("[Lidar Process] Hardware online.")
        except Exception as e:
            self.logging.error(f"[Lidar Process] Initialization failed: {e}")
            self.lidar_obj = None
        
        super(processLidar, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        pass

    def process_work(self):
        pass

    def _init_threads(self):
        """Create and start the Lidar Reader and Detector threads."""
        # 1. Thread for raw data acquisition
        ReaderTh = threadReader(
            self.lidar_obj, 
            self.shared_container, 
            self.queuesList, 
            self.logging, 
            self.debugging
        )
        self.threads.append(ReaderTh)
        
        # 2. Thread for mathematical analysis of the point cloud
        DetectorTh = threadDetector(
            self.shared_container, 
            self.queuesList, 
            self.logging, 
            self.debugging
        )
        self.threads.append(DetectorTh)

    def stop(self):
        """Graceful shutdown: stop threads first so threadReader.stop() handles hardware teardown."""
        super(processLidar, self).stop()
