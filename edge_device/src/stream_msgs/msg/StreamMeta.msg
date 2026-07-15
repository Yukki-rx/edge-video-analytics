# Per-frame metadata for one encoded H.264 frame sent over RTP/UDP.
# header.stamp = capture time of the frame on the endpoint camera.
std_msgs/Header header
uint32 seq              # frame sequence number assigned by the sender
time sent_stamp         # when the encoded frame left the endpoint
uint32 bytes            # size of the encoded H.264 frame in bytes
float32 fps_setting     # sender target FPS at the time of sending
uint32 bitrate_kbps     # encoder bitrate setting at the time of sending
