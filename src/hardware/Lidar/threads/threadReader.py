# ==============================================================================
# THREAD FLOW DESCRIPTION:
# THIS THREAD HANDLES RAW DATA ACQUISITION FROM THE LDROBOT LD19 DTOF LIDAR
#
# INPUT:
#   - Serial stream from LD19 at /dev/ttyUSB0, 230400 baud.
#
# PACKET FORMAT (47 bytes):
#   Byte  0     : Header   0x54
#   Byte  1     : VerLen   0x2C  (version=1, 12 points per packet)
#   Bytes 2-3   : Speed    uint16 LE  (deg/s × 100)
#   Bytes 4-5   : StartAngle uint16 LE (0.01°)
#   Bytes 6-41  : 12 × [Distance uint16 LE (mm), Intensity uint8]
#   Bytes 42-43 : EndAngle   uint16 LE (0.01°)
#   Bytes 44-45 : Timestamp  uint16 LE (ms)
#   Byte  46    : CRC8
#
# OUTPUT:
#   - shared_container['last_scan']:
#       {"data": [(intensity, angle_deg, distance_mm), ...], "timestamp": float}
#       Published once per full revolution (~10 Hz), covering all 360°.
#       Revolution boundary detected by start-angle wrap-around.
# ==============================================================================

import struct
import time
from src.templates.threadwithstop import ThreadWithStop

# ─── LD19 protocol constants ──────────────────────────────────────────────────
_LD19_HEADER   = 0x54
_LD19_VERLEN   = 0x2C   # 12 data points per packet
_LD19_POINTS   = 12
_LD19_PKT_SIZE = 47     # total bytes per packet

# CRC-8 lookup table (LDROBOT standard polynomial)
_CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]


def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


class threadReader(ThreadWithStop):
    """
    Reads raw 47-byte packets from the LD19 DTOF Lidar over serial and
    stores parsed point clouds in shared_container['last_scan'].
    """

    def __init__(self, serial_port, shared_container, queueList, logging, debugging=False):
        self.serial_port      = serial_port
        self.shared_container = shared_container
        self.queuesList       = queueList
        self.logging          = logging
        self.debugging        = debugging

        self.shared_container['last_scan'] = None

        self.subscribe()
        super(threadReader, self).__init__(pause=0.0001)

    def subscribe(self):
        pass

    def state_change_handler(self):
        pass

    # ── Packet I/O ────────────────────────────────────────────────────────────

    def _read_packet(self):
        """
        Synchronise to the next valid LD19 packet header and read 47 bytes.
        Returns the raw packet bytes, or None if the thread should stop.
        """
        while not self._blocker.is_set():
            # Wait for header byte
            b = self.serial_port.read(1)
            if not b or b[0] != _LD19_HEADER:
                continue

            # Read and verify VerLen
            vl = self.serial_port.read(1)
            if not vl or vl[0] != _LD19_VERLEN:
                continue

            # Read the remaining 45 bytes
            rest = self.serial_port.read(_LD19_PKT_SIZE - 2)
            if len(rest) < _LD19_PKT_SIZE - 2:
                continue

            packet = bytes([_LD19_HEADER, _LD19_VERLEN]) + rest

            # Discard packets with bad CRC
            if _crc8(packet[:-1]) != packet[-1]:
                if self.debugging:
                    self.logging.warning("[LiDAR Reader] CRC mismatch — packet discarded.")
                continue

            return packet

        return None   # thread stop requested

    def _parse_packet(self, packet: bytes):
        """
        Decode one LD19 packet into a list of (intensity, angle_deg, distance_mm).
        Angles are linearly interpolated between start_angle and end_angle.
        Zero-distance points (sensor out-of-range) are dropped.
        """
        start_angle = struct.unpack_from('<H', packet, 4)[0] / 100.0
        end_angle   = struct.unpack_from('<H', packet, 42)[0] / 100.0

        # Handle the 360° → 0° wraparound
        if end_angle < start_angle:
            end_angle += 360.0

        step = (end_angle - start_angle) / (_LD19_POINTS - 1)

        points = []
        for i in range(_LD19_POINTS):
            offset    = 6 + i * 3
            distance  = struct.unpack_from('<H', packet, offset)[0]   # mm
            intensity = packet[offset + 2]
            angle     = (start_angle + i * step) % 360.0

            if distance > 0:
                points.append((intensity, angle, float(distance)))

        return points

    # ── Main loop ─────────────────────────────────────────────────────────────

    def thread_work(self):
        """Accumulate full LD19 revolutions and publish complete scans.

        One revolution (~12 packets, ~144 points covering 360°) is detected
        by a start-angle wrap-around. Publishing a full scan instead of one
        packet guarantees that threadDetector always sees every angular sector.
        """
        if self.serial_port is None:
            self.logging.error("[LiDAR Reader] No serial port — cannot acquire data.")
            time.sleep(1.0)
            return

        try:
            current_scan = []
            prev_start_angle = None

            while not self._blocker.is_set():
                packet = self._read_packet()
                if packet is None:
                    break   # stop requested

                start_angle = struct.unpack_from('<H', packet, 4)[0] / 100.0
                points = self._parse_packet(packet)

                if prev_start_angle is not None and (prev_start_angle - start_angle) > 180.0:
                    # True revolution wrap: angle dropped >180° (e.g. 355°→5°).
                    # Minor non-monotonic jitter within a revolution is ignored.
                    if current_scan:
                        self.shared_container['last_scan'] = {
                            "data":      current_scan,
                            "timestamp": time.perf_counter(),
                        }
                        if self.debugging:
                            self.logging.info(
                                f"[LiDAR Reader] Full scan — {len(current_scan)} points.")
                    current_scan = list(points)
                else:
                    current_scan.extend(points)

                prev_start_angle = start_angle

        except Exception as e:
            self.logging.error(f"[LiDAR Reader] Serial error: {e}")
            time.sleep(0.5)

    def stop(self):
        """Signal the read loop to exit."""
        super(threadReader, self).stop()
