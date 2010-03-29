Header header
string name         # joint name
int32[] motor_ids   # motor ids controlling this joint

float64 goal_pos    # commanded position (in radians)
float64 current_pos # current joint position (in radians)
float64 error       # error between commanded and current positions (in radians)
float64 velocity    # current joint speed (in radians per second)
float64 load        # current load
bool is_moving      # is joint currently in motion

