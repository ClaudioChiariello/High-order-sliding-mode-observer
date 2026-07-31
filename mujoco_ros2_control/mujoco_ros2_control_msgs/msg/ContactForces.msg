# ContactForceArray.msg
std_msgs/Header header

string[] body1_names
string[] body2_names

geometry_msgs/Point[] contact_positions
geometry_msgs/Vector3[] forces
geometry_msgs/Vector3[] torques