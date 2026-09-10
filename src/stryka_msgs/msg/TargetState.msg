# Estimated state of the intercept target.
#
# Produced by the tracker, consumed by guidance and the mission manager.
# Every field here is in SI units and world-frame geometry. Guidance never
# sees pixels: the detector's bounding box is converted to an angle using
# f_px upstream, and only radians/metres cross this boundary.

std_msgs/Header header

bool     valid                 # false when the tracker is coasting past its limit
float64  age_seconds           # wall-clock age of the last real detection, seconds

geometry_msgs/Point    position          # world frame, relative to the vehicle, metres
geometry_msgs/Vector3  velocity          # world frame, metres/second
float64[9] position_covariance           # row-major 3x3, metres^2

float64  range_estimate        # line-of-sight range to target, metres
float64  bearing_error         # angle off boresight, RADIANS -- never pixels
