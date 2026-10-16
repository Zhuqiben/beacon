std_msgs/Header header
string[]  colors        # landmark colour, parallel to the arrays below
float32[] x             # m, estimated world x
float32[] y             # m, estimated world y
float32[] sigma         # m, position uncertainty reported by the EKF
bool[]    valid         # true once the landmark has been initialised
