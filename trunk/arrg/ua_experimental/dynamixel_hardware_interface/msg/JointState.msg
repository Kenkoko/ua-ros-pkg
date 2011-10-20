Header header
string name             # joint name

int32[] motor_ids       # motor ids controlling this joint
int32[] motor_temps     # motor temperatures, same order as motor_ids

float64 target_position # commanded position (in radians)
float64 target_velocity # commanded position (in radians per second)
float64 position        # current joint position (in radians)
float64 velocity        # current joint speed (in radians per second)
float64 load            # current load
bool    moving          # is joint currently in motion

