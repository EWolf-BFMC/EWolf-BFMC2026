# ==============================================================================
# THREAD FLOW DESCRIPTION:
# THIS THREAD HANDLES LOW-LEVEL LIDAR DATA ACQUISITION
# 
# INPUT: 
#   - Name: Hardware Serial Stream (/dev/ttyUSB0)
#   - Format: Raw bytes from RPLidar A2M8
#   - Source: Physical Lidar Sensor
#
# PROCESSING:
#   - Buffering: Uses max_buf_meas=500 for Pi 5 stability.
#   - Acquisition: Continuously clears input and pulls scans via iter_scans.
#
# OUTPUT:
#   - Name: last_scan (Internal shared memory)
#   - Format: List of tuples [(quality, angle, distance), ...]
#   - Destination: threadObstacleDetector (Internal Process Memory)
# ==============================================================================

import time
from src.templates.threadwithstop import ThreadWithStop
from rplidar import RPLidar

class threadReader(ThreadWithStop):
    """
    This thread handles the raw data acquisition from the Lidar hardware.
    It ensures the serial buffer is constantly cleared to prevent lag.
    """

    def __init__(self, lidar_obj, shared_container, queueList, logging, debugging=False):
        """
        Args:
            lidar_obj (RPLidar): Initialized hardware object.
            shared_container (dict): Shared dictionary for inter-thread communication.
            queueList (dict): Dictionary of multiprocessing queues for logging.
            logging (logging): Logging object for error reporting.
            debugging (bool): Flag for debug mode.
        """
        self.lidar = lidar_obj
        self.shared_container = shared_container
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # Initialize container with empty state
        self.shared_container['last_scan'] = None
        
        # Clear buffer at startup to ensure a clean stream start.
        try:
            self.lidar.clear_input()
        except:
            self.logging.warning("[LiDAR Reader] Could not clear input at startup.")
        
        # We use a very low pause to keep the serial buffer empty
        self.subscribe()
        super(threadReader, self).__init__(pause=0.0001)

    def subscribe(self):
        """No external subscriptions needed for raw data acquisition."""
        pass

    def state_change_handler(self):
        """Handle system mode transitions if necessary[cite: 1131]."""
        pass

    def thread_work(self):
        """Main acquisition loop using the blocking iter_scans generator."""
        try:
            # iter_scans yields a full 360-degree scan roughly every 100ms (10Hz)
            for scan in self.lidar.iter_scans(max_buf_meas=500):
                if self.stopped():
                    break
                
                # DATA PACKAGING: Include a timestamp so the Detector knows if the data is stale.
                self.shared_container['last_scan'] = {
                    "data": scan,
                    "timestamp": time.perf_counter()
                }
                
        except Exception as e:
            self.logging.error(f"[LiDAR Reader] Hardware disconnect or serial error: {e}")
            
            # Set to None so threadDetector triggers emergency.
            self.shared_container['last_scan'] = None
            
            # Brief sleep to allow USB/Serial reset if possible
            time.sleep(0.5)

    def stop(self):
        """Clean shutdown to ensure the Lidar motor actually stops spinning."""
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except:
            pass
        super(threadReader, self).stop()