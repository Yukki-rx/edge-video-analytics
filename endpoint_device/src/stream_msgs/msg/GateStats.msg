# Cumulative statistics of the content-aware motion gate on the endpoint.
std_msgs/Header header
uint32 frames_in        # frames received from the camera
uint32 frames_passed    # frames forwarded to the encoder
float32 pass_ratio      # frames_passed / frames_in (cumulative)
float32 motion_ratio    # motion pixel ratio of the most recent frame
