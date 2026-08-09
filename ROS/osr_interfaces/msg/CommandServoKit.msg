# central command message for PCA9685 control node
bool setup false # set to true for initial channel setup messages
int32 channel_id # 0-15
int32 actuation_range 300 # degrees, only used for setup messages
int32[2] pulse_width_range [500,2500] # microseconds, only used for setup messages
int32 new_angle 
string src