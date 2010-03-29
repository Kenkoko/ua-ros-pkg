string name         # joint name
int32[] motor_ids   # motor ids controlling this joint

float64 goal        # last commanded position (in radians)
float64 angle       # current joint position (in radians)
float64 error       # error between commanded and current positions (in radians)
float64 speed       # current joint speed (in radians per second)
bool moving         # is joint currently in motion

