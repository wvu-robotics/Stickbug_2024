#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include "stdio.h"
#include "stdlib.h"
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>

/*====================================================== OVERVIEW ==================================================
*
*   maps the logitech joystick for manipulator control
*   Publishes both: Positon and velocity commands    
*    
*   for position it does a discrete step size indicated by "delta"
*   for velocity "delta" serves as the gain on the joystick input
*
*///============================================================= Global Variables ================================
double goal_pose[6] = {.648,.5,0,0,0,0}; // x y z R P Y
double delta[6] = {.001, .001, .001, .001, .001, .001}; // dx dy dz dR dP dY, adjust sign here to flip directions 

double arm_vel[6] = {0,0,0,0,0,0}; // used to control the velocity of the arm movement
bool end_effector_on = false;      // trigger end effector on or off

//=============================================================== Joystick Callback ================================
void joyCallback( const sensor_msgs::Joy& joy_msg) {

  // adjust speed of the manipulator
  //  double vel = (joy_msg.axes[2] + 1.0) / 2;
  //  for(int i = 0; i < 6; i++)
  //    delta[i] = vel*((delta[i] > 0) - (delta[i] < 0)); // keep sign and multiply by vel

  // position control ------------------------------------------------------ 
  goal_pose[0] = goal_pose[0] + delta[0] * joy_msg.axes[1];
  goal_pose[1] = goal_pose[1] + delta[1] * joy_msg.axes[0];
  goal_pose[2] = goal_pose[2] + delta[2] * joy_msg.buttons[0] - delta[2] * joy_msg.buttons[3];
  goal_pose[3] = goal_pose[3] + delta[3] * joy_msg.axes[3];
  goal_pose[4] = goal_pose[4] + delta[4] * joy_msg.axes[4];
  goal_pose[5] = goal_pose[5] + delta[5] * joy_msg.buttons[4] - delta[5] * joy_msg.buttons[5];


 // velocity control --------------------------------------------------------
   arm_vel[0] = delta[0]*joy_msg.axes[0];
   arm_vel[1] = delta[1]*joy_msg.axes[1];
   arm_vel[2] = delta[2]*joy_msg.buttons[2] - delta[2] * joy_msg.buttons[3];
   arm_vel[3] = delta[3]*joy_msg.axes[4];
   arm_vel[4] = delta[4]*joy_msg.axes[5];
   arm_vel[5] = delta[5]*joy_msg.axes[2];

 // end effector control -----------------------------------------------------
  // set end effector on or off 
  if (joy_msg.buttons[0] > .5)
    end_effector_on = true;
  else if (joy_msg.buttons[1] > .5)
    end_effector_on = false;
}

//========================================================== MAIN =======================================================

int main(int argc, char **argv)
{
  // ros node stuff
  ros::init(argc, argv, "manipulator_teleop");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  // publisher and subscribers
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("manipulator_goal", 10);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("manipulator_vel", 10);
  ros::Publisher ee_pub = n.advertise<std_msgs::Bool>("end_effector_on", 10);

  ros::Subscriber joy_sub = n.subscribe("joy", 10, joyCallback);

  // main loop--------------------------------------------
  while (ros::ok())
  {
    
    // fill msgs
    geometry_msgs::Pose manip_goal;
    manip_goal.position.x = goal_pose[0];
    manip_goal.position.y = goal_pose[1];
    manip_goal.position.z = goal_pose[2];
    tf2::Quaternion quat;
    quat.setRPY(goal_pose[3],goal_pose[4],goal_pose[5]);
    quat.normalize();
    manip_goal.orientation.x = quat.x();
    manip_goal.orientation.y = quat.y();
    manip_goal.orientation.z = quat.z();
    manip_goal.orientation.w = quat.w();

    geometry_msgs::Twist vel;
    vel.linear.x = arm_vel[0];
    vel.linear.y = arm_vel[1];
    vel.linear.z = arm_vel[2];
    vel.angular.x = arm_vel[3];
    vel.angular.y = arm_vel[4];
    vel.angular.z = arm_vel[5];

    std_msgs::Bool ee;
    ee.data =  end_effector_on;
    
    // publish messages
    pose_pub.publish(manip_goal);
    vel_pub.publish(vel);
    ee_pub.publish(ee);

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

