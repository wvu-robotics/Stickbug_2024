#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
//#include <roboteq_control/ActuatorOut.h>
#include "geometry_msgs/Twist.h"
#include "math.h"

/*########################################################################################################################################################################

                                                                      OVERVIEW
  ________________________________________________________________________________________________________________________________________________________________________

          x                                                               |     Motor angular velocity to motor signal mapping
          ^    [centered on robot (o)]                                    |                          (s)
          |                                   Drive train                 |                           ^
     y <--o z                                  _________                  |          motor max signal |  /
                                           /  /         \  \              |                           | /
         *all motors rotate so (+)        / \/ m2     m3 \/ \             |                           |/
          results in (+) z rotation         /     (o)     \     ---       |                 ------|---+---|------> (motor angular vel [rad/s])
                                            \             /      |        |    motor max back speed  /|   motor max forward speed
                                             \           /       | radius |                         / |
                                              \___m1____/        |        |       motor min singal /  |
                                                   |             |        |
                                                 -----          ---       |
                                                 |-|                      |
                                            wheel radius                  |
*/                                
//################################################################### GLOBAL VARS ####################################################################################
short m1 = 0;   // motor 1 signal 
short m2 =0;    // motor 2 signal 
short m3 = 0;   // motor 3 signal

// default paramter values
double max_speed = 6;     // maximum angular speed of the robot's motors [rad/s]
double wheel_radius = .10;  // wheel radius in [m]
double min_signal = -1000;  // lowest signal value accepted by motor controller (full reverse)
double max_signal = 1000;   // highest signal value accepted by motor controller (full forward)

double vel2sig = (max_signal - min_signal) / (2 * max_speed * wheel_radius); // linear trend between the motor velocity and motor controller signal[signal / (m/s)]


/*############################################################ VELOCITY CALLBACK FUNCTION ############################################################################
*
*  converts linear and angular velocity commmand to motor controller signal command
*
*/
void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
     
  double x = -msg->linear.x;  // x velocity [m/s]
  double y = msg->linear.y;  // y velocity [m/s]
  double w = msg->angular.z; // z angular velocity [rad/s]


  // kiwi drive kinematics
  double m1_vel = -y + w; // [m/s]
  double m2_vel = .5*y - (sqrt(3)/2)*x + w; // [m/s]
  double m3_vel = .5*y + (sqrt(3)/2)*x + w; // [m/s]

  // convert the motor linear velocities to signals
  m1 = (short) (vel2sig * m1_vel);
  m2 = (short)(vel2sig * m2_vel);
  m3 = (short) (vel2sig * m3_vel);

  // enforce motor control limits
  if(m1 > (short)max_signal) m1 = max_signal;
  if(m1 < (short)min_signal) m1 = min_signal;
  
  if(m2 > (short)max_signal) m2 = max_signal;
  if(m2 < (short)min_signal) m2 = min_signal;

  if(m3 > (short)max_signal) m3 = max_signal;
  if(m3 < (short)min_signal) m3 = min_signal;


}


/*################################################################## MAIN #################################################################################
*
*   subscribes to "cmd_vel" of type geometry_msgs::twist
*   converts robot velocity to motor angular velocity
*   converts motor angular velocity to motor controller signal 
*   publishes to "motor_pow" of type std_msgs::Int16MultiArray where [m1,m2,m3] are the motor controller signals for motors 1,2, and 3 respectively
*/
int main(int argc, char **argv)
{
  
  // ros stuff
  ros::init(argc, argv, "kiwi_drive_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();

  // ros parameters ------------------------------------------------------------------------
  if(ros::param::get(ns + "/max_speed",max_speed)==false)
	{
		ROS_WARN("No parameter %s/max_speed specified, USING DEFAULT %f",ns.c_str(),max_speed);
	}
  if(ros::param::get(ns + "/wheel_radius",wheel_radius)==false)
	{
		ROS_WARN("No parameter %s/wheel_radius specified, USING DEFAULT %f",ns.c_str(),wheel_radius);
	}
  if(ros::param::get(ns + "/min_signal",min_signal)==false)
	{
		ROS_WARN("No parameter %s/min_signal specified, USING DEFAULT %f",ns.c_str(),min_signal);
	}
  if(ros::param::get(ns + "/max_signal",max_signal)==false)
	{
		ROS_WARN("No parameter %s/max_signal specified, USING DEFAULT %f",ns.c_str(),max_signal);
	}


  // initialization-------------------------------------------------------------------------- 
  vel2sig = (max_signal - min_signal) / (2 * max_speed * wheel_radius); // linear trend between the motor velocity and motor controller signal[signal / (m/s)]

  // publisher and subscribers ------------------------------------------------------------
  ros::Publisher motor_pub = n.advertise<std_msgs::Int16MultiArray>("roboteq/cmd_input", 10);
  ros::Subscriber vel_sub = n.subscribe("cmd_vel", 10, velCallback);


  // main loop
  while (ros::ok())
  {
    
    // create and fill message
   // roboteq_control::ActuatorOut msg;
    //msg.motor_cmd = {m1,m3,m2};
    
    std_msgs::Int16MultiArray msg;
    msg.data = {m1,m2,m3};


    // publish message
    motor_pub.publish(msg);

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
