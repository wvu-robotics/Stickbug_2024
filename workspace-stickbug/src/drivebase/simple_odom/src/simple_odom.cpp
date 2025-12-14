#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "math.h"
#include <tf2/LinearMath/Quaternion.h>

/*################################################ OVERVIEW ###############################################################
* 
*               frame setup                         |      1. recieves the command velocity of the robot
*                                                   |      2. transforms the velocity from the robot frame to the odometry frame
*                     RX                            |      3. integrates the global velocity to the new global position
*                     ^                             |
*                     |  Yaw                        |      X = X0 + Xvel * dt
*      ^ Y     RY <--[^] ----> X                    |      Y = Y0 + Yvel * dt
*      |         robot_frame "base_footprint"       |    Yaw = Yaw0 + Yaw_rate * dt    
*      |                                            |
*      O----> X                                     |
*    Z                                              |
*   odometry_frame "odom"                           |
*                                                   |
*===========================================================================================================================
*
*  subscribes to "cmd_vel" of type geometry_msgs::Twist 
*  publishes to "odom"  of type nav_msgs::Odometry
*
*============================================================================================================================
*
* NOTE: this should only be used to test and debug code when a quick odometry topic is needed. THIS DOES NOT PROVIDE GOOD
*       LOCALIZATION, it is a simple integral of command velocity THAT IS ASSUMED TO BE CHECKED BY ENCODERS
*
*///################################################# GLOBAL VARIABLES ######################################################
double X = 0;    // global x position
double Y = 0;    // global y position
double Yaw = 0;  // global heading

double u = 0;   // x velocity in robot frame
double v = 0;   // y velocity in robot frame
double r = 0;   // yaw rate in robot frame

double old_time = 0; // previous time
double dt = 0;       // change in time 

//################################################# HELPER FUNCTIONS #####################################################
void kinematics(){

    // convert velocity from robot frame to global frame and integrate over time
    X += u*dt*cos(Yaw) - v*dt*sin(Yaw);
    Y += u*dt*sin(Yaw) + v*dt*cos(Yaw);
    Yaw += r*dt;
}

//############################################################# CALLBACK FUNCITONS #######################################
void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   u = msg->linear.x;
   v = msg->linear.y;
   r = msg->angular.z;

   double curr_time = ros::Time::now().toSec();
   dt = curr_time - old_time;
   old_time = curr_time;
   kinematics();
}

//############################################################### MAIN #####################################################
int main(int argc, char **argv)
{
 
  // initalize ros node 
  ros::init(argc, argv, "simple_odom");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  // publishers and subcribers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_simple", 10);
  ros::Subscriber vel_sub = n.subscribe("cmd_vel", 10, velCallback);
  
  // set initial time
  old_time = ros::Time::now().toSec();

  while (ros::ok())
  {
   
   // covert the Euler Roll pitch yaw to a quaternion
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0,0,Yaw);
    myQuaternion= myQuaternion.normalize();

   // create and fill the odometery msg
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/odom";
    msg.child_frame_id = "/base_footprint";
    msg.pose.pose.position.x = X;
    msg.pose.pose.position.y = Y;
    msg.pose.pose.orientation.x = myQuaternion.x();
    msg.pose.pose.orientation.y = myQuaternion.y();
    msg.pose.pose.orientation.z = myQuaternion.z();
    msg.pose.pose.orientation.w = myQuaternion.w();
    msg.twist.twist.linear.x = u;
    msg.twist.twist.linear.y = v;
    msg.twist.twist.angular.z = r;

    // publish the odometery msg
    odom_pub.publish(msg);
    
    // loop 
    ros::spinOnce();
    loop_rate.sleep();
  
  }
  return 0;
}



