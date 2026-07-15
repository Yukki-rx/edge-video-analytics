# Feedback control from the edge device to the endpoint sender.
std_msgs/Header header
float32 target_fps          # requested sending frame rate
uint32 target_bitrate_kbps  # requested encoder bitrate
string reason               # human-readable reason, e.g. "tracking 2 objects"
