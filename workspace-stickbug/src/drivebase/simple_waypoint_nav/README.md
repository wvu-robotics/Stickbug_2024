# simple_waypoint_nav  
simple straight line holonomic waypoint navigation via PID controller   

![rqt_graph of the publisher](https://github.com/wvu-robotics/simple_waypoint_nav/blob/holonomic/simple_waypoint_nav/images/rqt_graph.png)  

"/waypoint" geometry_msgs::Pose 2d position and orientation goal   
"/odom"     nav_msgs::Odom current 2d position and orientation  
"/cmd_vel"  geometry_msgs::Twist 2d holonomic command velocity  
