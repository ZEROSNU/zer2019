Header header


# Tracker
# control_mode, is_auto, estop, steer are controlled(When estop, then brake also.) 
# speed, gear and brake are controlled in model-estimator

bool is_auto            # False: Human control   True: Auto mode
bool estop              # False: nomal mode      True: Estop mode
int8 gear
int16 brake
float32 speed
float32 steer
int64 encoder       #...
