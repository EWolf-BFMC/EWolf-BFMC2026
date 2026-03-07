# ==============================================================================
# THREAD FLOW DESCRIPTION:
# THIS THREAD ACTS AS THE 'ACTUATOR EXECUTION' LAYER (THE MUSCLES).
# 
# INPUT:
#   - Message Name: ControlAction
#   - Format: Dictionary {
#         "behavior": BehaviorState,   # Decided by threadFSM
#         "e_y": float,                # Cross-track error (meters)
#         "theta_e": float,            # Heading error (radians)
#         "speed": float,              # Target speed (m/s)
#         "timestamp": float           # Safety watchdog timestamp
#     }
#   - Source: threadFSM (via Gateway)
#
# PROCESSING:
#   - Mode Switching: Diverts logic between states
#   - execute_stanley: Implements the steering math.
#   - execute_parking: Placeholder for future maneuvering logic.
#
# OUTPUT:
#   - Name: SteerMotor (ID 2) and SpeedMotor (ID 1)
#   - Format: String (str) as required by the NUCLEO Serial Protocol.
#   - Destination: processSerialHandler (via Gateway) -> NUCLEO
# ==============================================================================

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    ControlAction, SpeedMotor, SteerMotor
)
from src.control.Control.threads.allStates import BehaviorState
import time
import math
import numpy as np

class threadControl(ThreadWithStop):
    """
    This thread handles the physical actuation of the vehicle.
    It receives high-level decisions (ControlAction) and translates them
    into low-level serial commands for the NUCLEO board.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # --- Stanley Controller Parameters (normal lane following) ---
        self.k = 4.0
        self.ks = 0.2
        self.kd = 0.2

        # --- Stanley Controller Parameters (highway — higher speed, gentler gains) ---
        self.k_highway = 3.05   # Lower cross-track gain: less aggressive at high speed
        self.ks_highway = 0.5
        self.kd_highway = 0.67  # More damping: prevents oscillation at highway speed

        self.prev_steering_angle_rad = 0.0 # Memory for the derivative term
        
        # --- Calibration & Constraints ---
        self.max_steer_deg = 25.0   #Max steering
        self.steering_bias_deg = 0.0 # Track-day adjustment for misalignment
        self.MAX_COMMAND_STALE_TIME = 0.2 # 200ms guard
        self._last_command = None         # Cache for brief gateway gaps

        self.subscribe()
        
        # Senders for the NUCLEO motor and steering actuators
        self.steerSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedSender = messageHandlerSender(self.queuesList, SpeedMotor)
        
        # Runs at 100Hz (0.01s) for high-fidelity control response
        super(threadControl, self).__init__(pause=0.01)

    def subscribe(self):
        """Initializes subscribers for the unified command from threadLogic."""
        self.commandSubscriber = messageHandlerSubscriber(
            self.queuesList, ControlAction, deliveryMode="lastOnly", subscribe=True
        )

    def state_change_handler(self):
        """Standard handler for system mode transitions."""
        pass

    def thread_work(self):
        """
        Main Loop: Behavior Execution.
        Executes the BehaviorState decided by threadFSM.
        """
        new_packet = self.commandSubscriber.receive()
        if new_packet:
            self._last_command = new_packet
        command_packet = self._last_command

        # FAIL-SAFE: Stop if FSM has never sent a command yet
        if not command_packet:
            self.send_commands(0.0, 0.0)
            return

        # Close the loop on logic freezes (stale cached command)
        msg_time = command_packet.get("timestamp")
        if not msg_time:
            self.send_commands(0.0, 0.0)
            return

        if (time.perf_counter() - msg_time) > self.MAX_COMMAND_STALE_TIME:
            if self.debugging:
                self.logging.warning("[Control] Logic command too stale! Halting.")
            self.send_commands(0.0, 0.0)
            return

        behavior = command_packet.get("behavior", BehaviorState.IDLE)

        # --- PRIORITY 0: EMERGENCY BRAKE — overrides everything ---
        if behavior == BehaviorState.EMERGENCY_BRAKE:
            self.prev_steering_angle_rad = 0.0  # Prevent recovery jerk
            self.send_commands(0.0, 0.0)
            return

        # --- PRIORITY 1: OPEN-LOOP OVERRIDE ---
        # FSM sets "override_steer" (degrees) for any state that requires
        # pre-computed steering (intersections, parking phases, full stops).
        # This bypasses the Stanley controller entirely.
        if "override_steer" in command_packet:
            self.execute_open_loop(command_packet)
            return

        # --- PRIORITY 2: STANLEY CONTROL (active driving states) ---
        if behavior in (
                BehaviorState.LANE_FOLLOWING,
                BehaviorState.HIGHWAY_DRIVING,
                BehaviorState.ROUNDABOUT,
                BehaviorState.DECELERATING):
            self.execute_stanley(command_packet)

        # --- UNKNOWN STATE SAFETY FALLBACK ---
        else:
            self.send_commands(0.0, 0.0)
        
    # ================================ ALGORITHMS ========================================

    def execute_stanley(self, data):
        """
        Calculates steering angle using the Stanley Control Law for lane following.
        Input data expected: {'e_y', 'theta_e', 'speed'}
        """
        try:
            # Extract unified data (Determined by threadLogic)
            e_y = data.get('e_y', 0.0)
            theta_e = data.get('theta_e', 0.0)
            v = data.get('speed', 0.0)
            behavior = data.get('behavior', BehaviorState.LANE_FOLLOWING)

            # Select gains based on driving state
            if behavior == BehaviorState.HIGHWAY_DRIVING:
                k, ks, kd = self.k_highway, self.ks_highway, self.kd_highway
            else:
                k, ks, kd = self.k, self.ks, self.kd

            # If the car is stopped, keep wheels straight to avoid servo wear
            if v < 0.01:
                self.prev_steering_angle_rad = 0.0 # Reset while stopped
                self.send_commands(0.0, 0.0)
                return

            # Stanley Law
            steering_adj = math.atan2(k * e_y, v + ks)
            desired_rad = theta_e + steering_adj

            # Approximate Derivative Damping (subtracts rate-of-change to resist fast swings)
            diff = desired_rad - self.prev_steering_angle_rad
            final_rad = desired_rad - (kd * diff)
            self.prev_steering_angle_rad = final_rad    #Update memory for next frame
            
            # Internal Math Clamp
            steer_deg = math.degrees(final_rad)
            steer_deg = max(min(steer_deg, self.max_steer_deg), -self.max_steer_deg)
            
            # DIAG: log every 50th call (~0.5s at 100Hz) to observe sign correlation
            self._stanley_diag = getattr(self, '_stanley_diag', 0) + 1
            if self._stanley_diag % 50 == 1:
                #self.logging.warning(f"[Stanley] e_y={e_y:.4f}m | steer={steer_deg:.1f}deg")
                pass

            self.send_commands(v, steer_deg)

        except Exception as e:
            self.logging.error(f"Stanley Error: {e}")

    def execute_open_loop(self, data):
        """
        Dispatches FSM-precomputed speed and steering directly to the NUCLEO,
        bypassing the Stanley controller.

        Used for: intersection open-loop phases, parking sequences, and all
        full-stop states (IDLE, STOP_ACTION) where override_steer=0.0 is sent.
        Resets prev_steering_angle_rad whenever speed is zero so the derivative
        term does not produce a wheel-snap on the next motion command.
        """
        speed = data.get("speed", 0.0)
        steer_deg = data.get("override_steer", 0.0)
        if abs(speed) < 0.01:
            self.prev_steering_angle_rad = 0.0
        self.send_commands(speed, steer_deg)

    # ================================ HARDWARE DISPATCH =================================

    def send_commands(self, speed_m_s, steer_deg):
        """
        Scales metric values to the specific units required by the NUCLEO firmware.
        - Speed: m/s -> mm/s (integer string)
        - Steer: degrees -> deci-degrees (integer string)
        """
        try:
            # Fixes physical camera/servo tilt without re-tuning Stanley
            steer_deg += self.steering_bias_deg

            # Scale Speed: (e.g., 0.3 m/s -> 300 mm/s)
            speed_mm_s = int(speed_m_s * 1000) 
            speed_mm_s = np.clip(speed_mm_s, -500, 500) 

            # Scale Steering: (e.g., 25.0 deg -> 250 deci-degrees)
            # This allows the NUCLEO to handle 0.1 degree precision.
            steer_decideg = int(round(steer_deg * 10))
            steer_decideg = np.clip(steer_decideg, -250, 250)

            # DISPATCH to NUCLEO as strings
            self.speedSender.send(str(speed_mm_s))
            self.steerSender.send(str(steer_decideg))

            if self.debugging:
                # Log the actual values being sent to serial
                self.logging.info(f"HARDWARE | V: {speed_mm_s} mm/s | S: {steer_decideg} d-deg")

        except Exception as e:
            self.logging.error(f"Failed to format hardware commands: {e}")