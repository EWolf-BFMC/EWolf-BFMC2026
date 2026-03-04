# ==============================================================================
# THREAD FLOW DESCRIPTION:
# THIS THREAD PROCESSES LIDAR DATA TO DETECT OBSTACLES IN FRONT OF THE VEHICLE
# 
# INPUT: 
#   - Name: last_scan (Internal shared memory dictionary)
#   - Format: {"data": [(quality, angle, distance), ...], "timestamp": float}
#   - Source: threadReader (Internal Process Memory)
#
# PROCESSING:
#   - Freshness Guard: Validates that data is < 300ms old to detect sensor freezes.
#   - Range Filtering: Extracts points only in the 30° front arc (255° to 285°).
#   - Noise Reduction: Confirms obstacle only if at least 3 points are detected in ROI.
#   - Reliability Logic: Reports 0.0 reliability on hardware failure or stale data.
#
# OUTPUT:
#   - Name: LidarObstacle
#   - Format: Dictionary {"distance": float, "reliability": float}
#   - Destination: threadLogic (The FSM) via Gateway
# ==============================================================================

import time
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import LidarObstacle 

class threadDetector(ThreadWithStop):
    """
    This thread analyzes the point cloud data provided by the threadReader.
    It filters the data to find valid obstacles within the vehicle's path.
    """

    def __init__(self, shared_container, queueList, logging, debugging=False):
        """
        Args:
            shared_container (dict): Shared dictionary to access the latest Lidar scan.
            queueList (dict): Dictionary of multiprocessing queues for message transmission.
            logging (logging): Logging object for system reports.
            debugging (bool): Flag for enabling console debug prints.
        """
        self.shared_container = shared_container
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # FSM makes the final stop decision
        self.MAX_STALE_TIME = 0.3  # 300ms before we consider the Lidar "frozen"
        
        self.obstacleSender = messageHandlerSender(self.queuesList, LidarObstacle)
        
        # 20Hz (pause=0.05) we react faster than the 10Hz Lidar spin
        super(threadDetector, self).__init__(pause=0.05)

    def subscribe(self):
        """No external subscriptions needed; data is pulled from shared_container."""
        pass

    def thread_work(self):
        """Processes scan with freshness and failure logic."""
        # 1. ACQUISITION
        scan_packet = self.shared_container.get('last_scan')
        
        # 2. Lidar Disconnected
        if scan_packet is None:
            # If the reader signals None, we report a 0mm obstacle to force a stop
            self.obstacleSender.send({"distance": 0.0, "reliability": 0.0})
            if self.debugging:
                print("[LiDAR Detector] CRITICAL: Lidar stream lost!")
            return

        try:
            # 3. Lidar Frozen
            scan_data = scan_packet.get("data", [])
            timestamp = scan_packet.get("timestamp", 0)
            
            if (time.perf_counter() - timestamp) > self.MAX_STALE_TIME:
                # Data is too old. Report danger and zero reliability.
                self.obstacleSender.send({"distance": 0.0, "reliability": 0.0})
                if self.debugging:
                    print("[LiDAR Detector] WARNING: Stale data detected!")
                return

            # 4. FRONT ARC FILTERING (255° to 285°)
            # LD19 mounting: cable connector faces 90° (rear), so forward = 270°.
            # ±15° gives a 30° forward arc: 255°–285°.
            front_points = [
                dist for (_, angle, dist) in scan_data
                if (angle >= 255 and angle <= 285) and dist > 0
            ]

            # 5. NOISE REDUCTION & DETERMINISTIC OUTPUT
            # We send a message EVERY cycle so the FSM knows the path is CLEAR.
            if len(front_points) >= 3:
                closest_dist = min(front_points)
                self.obstacleSender.send({"distance": closest_dist, "reliability": 1.0})
                
                if self.debugging and closest_dist < 1000.0:
                    print(f"[LiDAR Detector] Obstacle at: {closest_dist:.2f} mm")
            else:
                # No obstacle found. Send "infinity" to signal a clear path.
                self.obstacleSender.send({"distance": float('inf'), "reliability": 1.0})
            
        except Exception as e:
            self.logging.error(f"[LiDAR Detector] Error processing scan: {e}")
            self.obstacleSender.send({"distance": 0.0, "reliability": 0.0})