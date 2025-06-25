# Emotion.msg
# Discrete facial-expression cue for the robot’s face display node.
# The `state` field MUST take one of the enumerated constants.

std_msgs/Header header      # time stamp for synch/debug (optional consumer)

uint8 NEUTRAL = 0           # baseline / idle
uint8 HAPPY   = 1           # success feedback
uint8 SAD     = 2           # failure feedback

uint8 state                 # chosen constant above
