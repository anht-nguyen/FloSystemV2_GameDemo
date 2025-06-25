# PoseScore.msg
# Simple yes/no outcome from the vision module plus an optional similarity
# index for future use.  A Header is included so you can correlate the score
# with the source image frame if desired.

std_msgs/Header header   # timestamp of the image on which the score is based
bool matched             # true  ⇒ player pose satisfies the rule
float32 similarity       # 0.0–1.0   (optional; 1.0 = perfect match)
