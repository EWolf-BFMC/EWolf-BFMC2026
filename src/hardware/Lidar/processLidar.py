# ==============================================================================
# PROCESS DESCRIPTION:
# THIS PROCESS MANAGES THE LD19 DTOF LIDAR SENSOR AND OBSTACLE DETECTION LOGIC
#
# THREADS:
#   - threadReader:   Raw packet acquisition from LD19 over serial (230400 baud).
#   - threadDetector: Analyses the point cloud to find frontal obstacles.
#
# SHARED RESOURCES:
#   - shared_container: {'last_scan': list | None}  zero-latency inter-thread data.
#   - serial_port:      pyserial Serial object shared with threadReader.
# ==============================================================================

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import serial
from src.templates.workerprocess import WorkerProcess
from src.hardware.Lidar.threads.threadReader import threadReader
from src.hardware.Lidar.threads.threadDetector import threadDetector

_LIDAR_PORT     = '/dev/ttyUSB0'
_LIDAR_BAUDRATE = 230400


class processLidar(WorkerProcess):
    """Manages the LDROBOT LD19 DTOF Lidar.

    Args:
        queueList  (dict): Shared multiprocessing queues.
        logging:           Logger object.
        ready_event:       Optional multiprocessing.Event signalled when ready.
        debugging  (bool): Verbose logging flag.
    """

    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList  = queueList
        self.logging     = logging
        self.debugging   = debugging

        self.shared_container = {'last_scan': None}

        try:
            self.serial_port = serial.Serial(
                port=_LIDAR_PORT,
                baudrate=_LIDAR_BAUDRATE,
                timeout=1.0,
            )
            self.logging.info(
                f"[Lidar Process] LD19 online on {_LIDAR_PORT} @ {_LIDAR_BAUDRATE} baud.")
        except Exception as e:
            self.logging.error(f"[Lidar Process] Initialization failed: {e}")
            self.serial_port = None

        super(processLidar, self).__init__(self.queuesList, ready_event)

    def state_change_handler(self):
        pass

    def process_work(self):
        pass

    def _init_threads(self):
        """Create threadReader (acquisition) and threadDetector (analysis)."""
        ReaderTh = threadReader(
            self.serial_port,
            self.shared_container,
            self.queuesList,
            self.logging,
            self.debugging,
        )
        self.threads.append(ReaderTh)

        DetectorTh = threadDetector(
            self.shared_container,
            self.queuesList,
            self.logging,
            self.debugging,
        )
        self.threads.append(DetectorTh)

    def stop(self):
        """Graceful shutdown: stop threads first, then close the serial port."""
        super(processLidar, self).stop()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
