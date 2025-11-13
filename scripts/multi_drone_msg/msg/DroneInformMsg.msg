# DroneState.msg
# Target and current positions in both local and global coordinate frames
LocalAndGlobalCoordinatesMsg target_position
LocalAndGlobalCoordinatesMsg current_position

# Drone configuration parameters
DroneParamsMsg params

# Current and previous state machine states
string current_state
string last_state
