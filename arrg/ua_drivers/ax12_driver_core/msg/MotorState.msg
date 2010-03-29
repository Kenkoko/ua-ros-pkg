int32 id            # motor id
int32 goal          # commanded position (in ax12 encoder units [0-1023])
int32 position      # current position (in ax12 encoder units [0-1023])
int32 error         # difference between current and goal positions
int32 speed         # current speed
int32 load          # current load
float64 voltage     # current voltage
int32 temperature   # current temperature
bool moving         # whether the motor is currently in motion
