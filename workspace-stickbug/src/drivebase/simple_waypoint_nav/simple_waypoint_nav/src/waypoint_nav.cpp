#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "stdio.h"
#include "stdlib.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

using namespace std;

/*###########################################################  OVERVIEW  #############################################################################################
*
*                             robot setup                                          Proportional Integral Derrrivative (PID) control
*                                                    angular error     |                  
*                                                     |/               |   each dimension X, Y, Yaw has a PID controller that it uses to move to the waypoint
*                                   waypoint  -->     *  -----         |  
*                                                     |    |           |  to tune a  PID controller   
*        ^ X                 	        x_error         |    |           |  1. set K****_p to 1, K****_i = 0, K****_d = 0
*   Y    |                        |===================|    |           |  2. increae K****_p until it starts to overshoot the target 
*   <----O                        |                        | y_error   |  3. divide K****_p by 4,  it is now set
*         Z                  +---------+                   |           |  5. slowly increase K****_i, until there is no more residual error, it is now set
*                            |         |                   |           |  6. slowly increase K****_d, until the it starts to overshoot the target, set at prev value
*            ROBOT -->       |    O    | --------------------          |
*                            |         |                               |
*                            +---------+                               |
*                                                                      |
*=======================================================================================================================================================================
*
*  subscribes to "odom" of type nav_msgs::odometry  as the current position of the robot and provides the time increments
*  subscribes to "waypoint" of type geometry_msgs::PoseStamped as the target position of the robot 
*  publishes  to "cmd_vel"  of type geometry_msgs::Twist as the desired velocity of the robot
*
*///########################################################  GLOBAL VARIABLES  #############################################
double Kdist_p = .5;           // proportional gain for linear motion
double Kdist_i = .0001;        // integral gain for linear motion
double Kdist_d = .01;          // derivative gain for linear motion

double x_error = 0;            // current difference between target x position and current x position
double prev_x_error = 0;       // the previous time step x_error
double total_x_error = 0;      // the sum of x_error over time since the waypoint goal was recieved
double x_error_rate = 0;       // time rate of change of x_error  i.e. d(x_error)/dt

double y_error = 0;            // current difference between target y position and current y position
double prev_y_error = 0;       // the previous time step y_error
double total_y_error = 0;      // the sum of y_error over time since the waypoint goal was recieved
double y_error_rate = 0;       // time rate of change of y_error  i.e. d(y_error)/dt

double Kang_p = .1;            // proportional gain for angular motion
double Kang_i = .0001;         // integral gain for angular motion
double Kang_d = .01;           // derrivative gain for angular motion

double ang_error = 0;          // current difference between target heading and current heading
double prev_ang_error = 0;     // the previous time step ang_error
double total_ang_error = 0;    // the sum of ang_error over time since the waypoint goal was recieved
double ang_error_rate = 0;     // time rate of change of ang_error  i.e. d(ang_error)/dt

double waypoint[3] = {0,0,0};  // target pose to achieve [x, y, yaw] [m, m, rad]
double pose[3] = {0,0,0};      // current pose [x, y, yaw] [m, m, rad]
double max_vel = .5;           // maximum linear velocity in one direction 
double max_ang_vel = 2.3;      // maximum angular velocity 

double old_time_sec = 0;       // previous time step, time value full second component [sec]      
double old_time_nsec = 0;      // previous time step, time value nano second component [nsec]
double dt = 0;                 // change in time from previous time step to now

bool got_odom = false;         // flag to make sure we got an odometry message
bool got_waypoint = false;     // flag to make sure that we got a waypoint message

/*======================================================================  CALLBACK FUNCTIONS =======================================================================
*
*       waypoint callback  --> recieves a geometry_msgs::PoseStamped message and sets it to the target x, y, and yaw position
*       odometry callback  --> recieves a nav_msgs::Odometry message and sets it to the current x, y, and yaw position
*
*///---------------------------------------------------------------------- get waypoint ---------------------------------
void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
 waypoint[0] = double(goal->pose.position.x);
 waypoint[1] = double(goal->pose.position.y);

 tf::Quaternion quat;
 tf::quaternionMsgToTF(goal->pose.orientation, quat);
 // the tf::Quaternion has a method to acess roll pitch and yaw
 double roll, pitch, yaw;
 tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 
 waypoint[2] = double(yaw);

 
  ROS_INFO("got waypoint x = %f  y = %f  yaw= %f", waypoint[0], waypoint[1], waypoint[2]);
 got_waypoint = true;
}
//------------------------------------------------------------------------- get current position --------------------------------------------

void odomCallback(const nav_msgs::Odometry::ConstPtr& position)
{
 pose[0] = position->pose.pose.position.x;
 pose[1] = position->pose.pose.position.y;

 // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(position->pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 
  pose[2] = double(yaw);
  
  dt = position->header.stamp.sec - old_time_sec + (position->header.stamp.nsec - old_time_nsec)/1000000000;
  old_time_sec = position->header.stamp.sec;
  old_time_nsec = position->header.stamp.nsec;
  
  got_odom = true;
  ROS_INFO("got odometry X = %f, Y= %f, Yaw = %f, dt = %f", pose[0], pose[1], pose[2], dt);
}
/*============================================================================================= HELPER FUNCTIONS =========================================================
*
*    x_velocity   --> creates the PID control x  linear velocity based on the x errors [m/s]
*    y_velocity   --> creates the PID control y linear velocity based on the y errors  [m/s]
*    angular_velocity   --> creates the PID control yaw velocity based on the yaw errors   [rad/s]
*    calculate_errors   --> calculates the proportional, integral, and derrivative errors for the PID controllers for x, y, yaw
*
*///----------------------------------------------------------------------------- control velocity in x direction
double x_velocity(){
  
  double vel = Kdist_p*x_error + Kdist_i*total_x_error + Kdist_d*x_error_rate;
	
  if(vel > max_vel){
    vel = max_vel;	
  }
  else if(vel < -max_vel){
    vel = -max_vel;
  }

  return vel;

}
//--------------------------------------------------------------------------------- control velocity in y direction
double y_velocity(){
  
  double vel = Kdist_p*y_error + Kdist_i*total_y_error + Kdist_d*y_error_rate;
	
  if(vel > max_vel){
    vel = max_vel;	
  }
  else if(vel < -max_vel){
    vel = -max_vel;
  }

  return vel;

}
//--------------------------------------------------------------------------------- control angular velocity in yaw direction
double angular_velocity(){
  
  double vel = Kang_p*ang_error + Kang_i*total_ang_error + Kang_d*ang_error_rate;
	
  if(vel > max_ang_vel){
    vel = max_ang_vel;	
  }
  else if(vel < -max_ang_vel){
    vel = -max_ang_vel;
  }

  return vel;

}
//--------------------------------------------------------------------------------- control errors calculator 
void calculate_errors(){

  // x errors
  x_error = waypoint[0]-pose[0];
  total_x_error += x_error;
  x_error_rate = (x_error - prev_x_error)/dt;
  prev_x_error = x_error;

 // y errors
  y_error = waypoint[1]-pose[1];
  total_y_error += y_error;
  y_error_rate = (y_error - prev_y_error)/dt;
  prev_y_error = y_error;

   // heading errors
  ang_error = waypoint[2]-pose[2];
  total_ang_error += ang_error;
  ang_error_rate = (ang_error - prev_ang_error)/dt;
  prev_ang_error = ang_error;

}

//================================================================== MAIN =====================================================================

int main(int argc, char **argv)
{
 
  // start a ros node 
  ros::init(argc, argv, "waypoint_nav");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();

  // ros parameters ----------------------------------------------------------------
  if(ros::param::get(ns + "/Kdist_p",Kdist_p)==false)
	{
		ROS_WARN("No parameter %s/Kdist_p specified, USING DEFAULT %f",ns.c_str(),Kdist_p);
	}
  if(ros::param::get(ns + "/Kdist_i",Kdist_i)==false)
	{
		ROS_WARN("No parameter %s/Kdist_i specified, USING DEFAULT %f",ns.c_str(),Kdist_i);
	}
  if(ros::param::get(ns + "/Kdist_d",Kdist_d)==false)
	{
		ROS_WARN("No parameter %s/Kdist_d specified, USING DEFAULT %f",ns.c_str(),Kdist_d);
	}
  if(ros::param::get(ns + "/Kang_p",Kang_p)==false)
	{
		ROS_WARN("No parameter %s/Kang_p specified, USING DEFAULT %f",ns.c_str(),Kang_p);
	}
  if(ros::param::get(ns + "/Kang_i",Kang_i)==false)
	{
		ROS_WARN("No parameter %s/Kang_i specified, USING DEFAULT %f",ns.c_str(),Kang_i);
	}
  if(ros::param::get(ns + "/Kang_d",Kang_d)==false)
	{
		ROS_WARN("No parameter %s/Kang_d specified, USING DEFAULT %f",ns.c_str(),Kang_d);
	}
  if(ros::param::get(ns + "/max_vel",max_vel)==false)
	{
		ROS_WARN("No parameter %s/max_vel specified, USING DEFAULT %f",ns.c_str(),max_vel);
	}
  if(ros::param::get(ns + "/max_ang_vel",max_ang_vel)==false)
	{
		ROS_WARN("No parameter %s/max_ang_vel specified, USING DEFAULT %f",ns.c_str(),max_ang_vel);
	}

  // instantiation -----------------------------------------------------------------


  // set up subcribers and publishers
  ros::Subscriber waypoint_sub = nh.subscribe("waypoint", 10, waypointCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);	
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  // main loop
  while (ros::ok())
  {
   
   if(got_odom && got_waypoint){ // check to make sure we got a waypoint and pose
     calculate_errors(); // calculate PID control errors for x, y, and yaw

     geometry_msgs::Twist msg;    //create a command velocity and fill it with the control velocities
     msg.linear.x = x_velocity();
     msg.linear.y = y_velocity();
     msg.angular.z = angular_velocity();
     vel_pub.publish(msg);
     ROS_INFO("at dawn we march to take back the holy land!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
     got_odom = false;  // reset the odometry flag
    }
    else{
     ROS_INFO("ERROR 404 holy land not found????????????????????????????????????????????????");	
    }
    
    // ros check for topics and loop
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}









