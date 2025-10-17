# Trakstar message definition
# Contains transform data for multiple trackers
# TODO: Consider using custom message type instead of geometry_msgs/TransformStamped

std_msgs/Header header
int32 n_tracker
geometry_msgs/Transform[] transform
