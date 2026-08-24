std_msgs/Header header
float64[] raw_values
float64[] filtered_values
geometry_msgs/Vector3 force
float64 force_magnitude
bool contact
bool warning
bool stop
bool valid
string diagnostic_status
float64 clearance_m
string source

uint8 REGION_NO_CONTACT=0
uint8 REGION_CONTACT=1
uint8 REGION_WARNING=2
uint8 REGION_STOP=3
uint8 region
