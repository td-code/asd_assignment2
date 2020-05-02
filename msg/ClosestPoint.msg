std_msgs/Header header # standard header, cf. http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
bool    valid          # True if a closest point has been found, 
                       # False if no obstacle has been detected
float64 distance_m     # the distance to the closest point [m]
float64 angle_rad      # the heading angle to the closes point [rad]