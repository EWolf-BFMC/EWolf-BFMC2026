from enum import Enum

# ==============================
# GLOBAL STATE DEFINITIONS
# ==============================

class BehaviorState(Enum):
    """
    Defines what the car is doing during autonomous navigation.
    These states are managed by threadFSM (The Brain).
    """
    IDLE            = 0  # Waiting for AUTO mode activation
    LANE_FOLLOWING  = 1  # City area navigation (Min 20 cm/s) 
    HIGHWAY_DRIVING = 2  # Highway area navigation (Min 40 cm/s)
    DECELERATING    = 3  # Visibly slowing down for Parking or Crosswalk
    STOP_ACTION     = 4  # 3-second halt (Stop Sign) or waiting for Red Light
    EMERGENCY_BRAKE = 5  # Immediate stop for pedestrians or obstacles
    PARKING_MANEUVER = 6 # Executing the found parking spot movement
    ROUNDABOUT      = 7  # Counter-clockwise navigation state
    INTERSECTION    = 8  # Handling 3- or 4-way junction logic

class SignType(Enum):
    """
    Standardized types for YOLO sign detection.
    """
    TRAFFIC_LIGHT    = 1  # RED, GREEN, or YELLOW
    STOP             = 2  # Halt 3 seconds before intersection
    PARKING          = 3  # Slow down and find empty spot
    CROSSWALK        = 4  # Slow down; stop if pedestrian intent is detected
    PRIORITY         = 5  # Enter intersection without halting
    HIGHWAY_ENTRY    = 6  # Switch to highway rules/speed
    HIGHWAY_EXIT     = 7  # Return to city rules/speed
    ONE_WAY          = 8  # Mandatory one-way road
    ROUNDABOUT       = 9  # Follow counter-clockwise navigation
    NO_ENTRY         = 10 # Prohibited entry road

class ObstacleZone(Enum):
    """
    Defines safety zones based on Lidar distance.
    Used for the gradual speed reduction logic.
    """
    CLEAR           = 0 # No obstacles (Full speed ahead)
    WARNING         = 1 # Obstacle detected (Proportional slowdown)
    DANGER          = 2 # Immediate collision risk (Emergency Brake)

class SpeedLimit(Enum):
    """
    Speed limitations (values in m/s).
    """
    CITY_MIN    = 0.20  # Minimum 20 cm/s in city 
    HIGHWAY_MIN = 0.40  # Minimum 40 cm/s in highway
    MAX_ALLOWED = 0.50  # Max for competition safety