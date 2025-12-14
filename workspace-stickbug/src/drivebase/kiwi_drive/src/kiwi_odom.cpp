#include "ros/ros.h"
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Dense>

# define M_PI           3.14159265358979323846
# define MAX_ENCODER_COUNTS 2147483648

using namespace std;
using namespace Eigen;

/*########################################################################################################################################################################

                                                                      OVERVIEW
                                             research paper https://www.tandfonline.com/doi/suppl/10.1080/00051144.2017.1391612?scroll=top
  ________________________________________________________________________________________________________________________________________________________________________

          x                                                               |    
          ^    [centered on robot (o)]                                    |     uses velocity transports of a ridgid body to determine the velocity of the robot            
          |                                   Drive train                 |     research paper https://www.tandfonline.com/doi/suppl/10.1080/00051144.2017.1391612?scroll=top
     y <--o z                                  _________                  |         
                                           /  /         \  \              |     [x_vel_global]                              [m1_ang_vel]
         *all motors rotate so (+)        / \/ m1     m3 \/ \             |     |y_vel_global| = [kinematic_matrix (3x3)] * |m2_ang_vel|  
          results in (+) z rotation         /     (o)     \     ---       |     [  yaw_rate  ]                              [m3_ang_vel]
                                            \             /      | robot  |   
                                             \           /       | radius |     global_pose = old_global_pose + global_velocity*dt 
                                              \___m2____/        |        |       
                                                   |             |        |
                                                 -----          ---       |
                                                 |-|                      |
                                            wheel radius                  |
*/                                
//################################################################### GLOBAL VARS ####################################################################################

double wheel_radius = .10;             // wheel radius in [m]
double robot_radius = .31;             // radius from geometric centroid of robot to center of wheel [m]
double beta = 60*M_PI/180;             // angle offset between the x axis and motors 2 and 3
double gear_ratio = 1;                 // gear ratio of the motor
double encoder_counts_per_revolution = 500;  // how many counts the encoder makes to make one full wheel rotation

VectorXd global_velocity(3);     // robot velocity in global frame {Vx [m/s], Vy[m/s], r[rad/s]}
MatrixXd kinematics_matrix(3,3); // converts from motor speeds to x, y, r global velocities
VectorXd motor_ang_velocity(3);  // motor speeds in [rad/s]  {m1,m2,m3}

double x = 0;    // current global x position of the robot
double y = 0;    // current global y positon  of the robot
double psi = 0;  // current global yaw angle of the robot

double prev_time = 0;  // previous time point
double curr_time = 0;  // current time point
double dt = 0;         // change in time

int32_t curr_encoder_counts[3] = {0,0,0};  // the current encoder counts of each motor 
int32_t prev_encoder_counts[3] = {0,0,0};  // the previous encoder counts of each motor
int32_t delta_counts[3] = {0,0,0};         // the change in encoder counts of each motor
int32_t flip_motor_direction[3] = {1,1,1}; // flips motor angular velocity direction, use 1 to keep direction, or -1 to flip 

// ########################################################################################### Helper functions #######################################################
// calculates the change in encoder counts, alloting for count overrun in both directions
int32_t deltaCounts(int32_t counts, int32_t counts_prev)
{  
  int64_t temp_counts = (int64_t)counts;
  int64_t temp_counts_prev = (int64_t)counts_prev;
  int64_t temp_delta_counts = temp_counts - temp_counts_prev;
  if(temp_delta_counts > MAX_ENCODER_COUNTS) // Wrap around going backwards
  {
    temp_counts += 2*MAX_ENCODER_COUNTS;
    temp_delta_counts = temp_counts - temp_counts_prev;
  }
  else if(temp_delta_counts < -MAX_ENCODER_COUNTS) // Wrap around going forwards
  {
    temp_counts -= 2*MAX_ENCODER_COUNTS;
    temp_delta_counts = temp_counts - temp_counts_prev;
  }
  return (int32_t)temp_delta_counts;
}

// updates the kinematics matrix that converts motor angular velocities to global velocites based on the current yaw of the robot
void update_kinematics_matrix(){

    kinematics_matrix(0,0) = wheel_radius*(sin(beta-psi)+sin(psi)) / (cos(2*beta)-1);
    kinematics_matrix(0,1) = wheel_radius*sin(psi)/(cos(beta)+1);
    kinematics_matrix(0,2) = wheel_radius*(sin(psi)-sin(beta+psi)) / (cos(2*beta)-1);

    kinematics_matrix(1,0) = wheel_radius*(cos(beta-psi)-cos(psi)) / (cos(2*beta)-1);
    kinematics_matrix(1,1) = -wheel_radius*cos(psi) / (cos(beta)+1);
    kinematics_matrix(1,2) = wheel_radius*(cos(beta+psi)-cos(psi)) / (cos(2*beta)-1);

    kinematics_matrix(2,0) = wheel_radius/(2*robot_radius*(cos(beta)+1));
    kinematics_matrix(2,1) = wheel_radius*cos(beta)/ (robot_radius*(cos(beta+1)));
    kinematics_matrix(2,2) = kinematics_matrix(2,0);

}

// updates the pose of the robot by integrating the velocity over the time change
void kinematics(){
    VectorXd curr_pose(3);
    VectorXd prev_pose(3);
    prev_pose(0) = x;
    prev_pose(1) = y;
    prev_pose(2) = psi;

    global_velocity = kinematics_matrix*motor_ang_velocity;
    curr_pose = prev_pose + global_velocity*dt;
    x = curr_pose(0);
    y = curr_pose(1);
    psi = curr_pose(2);

}

// callback function from 3 channel roboteq fills the motor angular velocity, encoder counts, and change in time
void RoboteqCallback(const hw_interface_plugin_roboteq::RoboteqData::ConstPtr& msg)
{
  for(int i = 1; i < 3; i++){
      prev_encoder_counts[i] = curr_encoder_counts[i];
      curr_encoder_counts[i] = msg->encoder_counter_absolute.at(i);
      delta_counts[i] = deltaCounts(curr_encoder_counts[i], prev_encoder_counts[i]);
      motor_ang_velocity(i) =  (double) msg->feedback.at(i)*(1.0/gear_ratio)*(2.0*M_PI)*(1.0/60.0)*flip_motor_direction[i];
    }
    prev_time = curr_time;
    curr_time = ros::Time::now().toSec();
    dt = curr_time - prev_time;
  
}


/*################################################################## MAIN #################################################################################
*
*   subscribes to "roboteq" of type hw_interface_plugin_roboteq::RoboteqData
*   reads in the motor angular velocity in rpm and converts to rad/s
*   converts motor angular velocity to global x, y, and yaw velocites 
*   publishes to "wheel_odom" of type nav_msgs::Odometry  TODO:: FILL IN COVARIENCE
*/
int main(int argc, char **argv)
{
  
  // ros stuff
  ros::init(argc, argv, "kiwi_odom_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();

  // ros parameters -----------------------------------------------------------
  if(ros::param::get(ns + "/wheel_radius",wheel_radius)==false)
	{
		ROS_WARN("No parameter %s/wheel_radius specified, USING DEFAULT %f",ns.c_str(),wheel_radius);
	}
  if(ros::param::get(ns + "/robot_radius",robot_radius)==false)
	{
		ROS_WARN("No parameter %s/robot_radius specified, USING DEFAULT %f",ns.c_str(),robot_radius);
	}
  if(ros::param::get(ns + "/beta",beta)==false)
	{
		ROS_WARN("No parameter %s/beta specified, USING DEFAULT %f",ns.c_str(),beta);
	}
  if(ros::param::get(ns + "/gear_ratio",gear_ratio)==false)
	{
		ROS_WARN("No parameter %s/gear_ratio specified, USING DEFAULT %f",ns.c_str(),gear_ratio);
	}
  if(ros::param::get(ns + "/encoder_counts_per_revolution",encoder_counts_per_revolution)==false)
	{
		ROS_WARN("No parameter %s/encoder_counts_per_revolution specified, USING DEFAULT %f",ns.c_str(),encoder_counts_per_revolution);
	}
  
  // initialization ----------------------------------------------------------

  // publisher and subscribers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel_odom", 10);
  ros::Subscriber roboteq_sub = n.subscribe("roboteq", 1, RoboteqCallback); 
 
   int seq = 0;
  // main loop
  while (ros::ok())
  {

    update_kinematics_matrix();
    kinematics();
    
    // create and fill odom message
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    // Compute odom message quaternion from yaw angle
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(psi);
    // Velocity (twist)
    odom_msg.twist.twist.linear.x = global_velocity(0);
    odom_msg.twist.twist.linear.y = global_velocity(1);
    odom_msg.twist.twist.angular.z = global_velocity(2);
    // Header and frame info
    odom_msg.header.seq = seq;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    // publish message
    odom_pub.publish(odom_msg);

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();
    seq++;

  }


  return 0;
}
