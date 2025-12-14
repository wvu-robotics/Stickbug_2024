#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>

#include "stdio.h"
#include "stdlib.h"

# define M_PI           3.14159265358979323846

using namespace std;

/*###################################################################### Overview #####################################################################################
 *                             
 *  Description: This node is used for general control over a simple gripper end effector, and includes open / close,    
 *  
 *  NOTE: it is ASSUMED that all closed loop control of the end effector is housed on an arduino or sub-computing device 
 * 
 *=====================================================================================================================================================================
 *                 TOP VIEW                           |           OVERVIEW
 *                                                    |
 *                  open angle                        |   1. subscribes to <namespace>/end_effector_on std_msgs::Boolean    
 *                    <-->     ---                    |   2. publishes  to <namespace>/ee_offset  std_msgs::Pose
 *     ^  X         \     /     |                     |   3. publishes  to <namespace>/ee_joints  sensor_msgs::JointState
 * Y   |             \ x /   gripper_length           |   
 * <---0              \ /       |                     |   NOTE: the Open angle and Closed angle are defined as the positive angle between both fingers of the gripper
 *      Z              O       ---                    |
 *                     ^                              |
 *                     +--- motor joint               |
 *                                                    |
 *    NOTE: 'x' is the actuation location             |
 *                                                    |
*///################################################################## Global Variables #################################################################################

double actuation_x = .02; // the dx position from the end effector base to the desired actuation location on the gripper
double actuation_y = 0;   // the dy position from the end effector base to the desired actuation location on the gripper
double actuation_z = 0;   // the dz position from the end effector base to the desired actuation location on the gripper

double open_angle = M_PI;                    // the angle considered open for the gripper
double close_angle = 0.0;                    // the angle considered closed for the gripper

double gripper_angle = 0.0;                  // the current gripper angle

//################################################################# Command Callback ##################################################################################

void eeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data) gripper_angle = close_angle;
    if(!msg->data) gripper_angle = open_angle;

}


//################################################################## MAIN #################################################################################

int main(int argc, char **argv)
{

  // ros node stuff
  ros::init(argc, argv, "gripper_ee_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();

  // ros params-------------------------------------------------------------------------------------------------------
  if(ros::param::get(ns + "/actuation_x",actuation_x)==false)
	{
		ROS_WARN("No parameter %s/actuation_x specified, USING DEFAULT %f",ns.c_str(),actuation_x);
	}
  if(ros::param::get(ns + "/actuation_y",actuation_y)==false)
	{
		ROS_WARN("No parameter %s/actuation_y specified, USING DEFAULT %f",ns.c_str(),actuation_y);
	}
  if(ros::param::get(ns + "/actuation_z",actuation_z)==false)
	{
		ROS_WARN("No parameter %s/actuation_z specified, USING DEFAULT %f",ns.c_str(),actuation_z);
	}
   if(ros::param::get(ns + "/open_angle",open_angle)==false)
	{
		ROS_WARN("No parameter %s/open_angle specified, USING DEFAULT %f",ns.c_str(),open_angle);
	}
  if(ros::param::get(ns + "/close_angle",close_angle)==false)
	{
		ROS_WARN("No parameter %s/close_angle specified, USING DEFAULT %f",ns.c_str(),close_angle);
	}
  // initalization ---------------------------------------------------------------------------------------------------
  double actuation_location[] = {actuation_x,actuation_y,actuation_z}; // houses the [dx,dy,dz] position from the end effector base to the desired actuation location on the gripper
  
  // publisher and subscribers
  ros::Publisher offset_pub = n.advertise<geometry_msgs::Pose>("ee_offset", 10);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("ee_joints", 10);
  ros::Subscriber ee_sub = n.subscribe("end_effector_on", 10, eeCallback);

  if (ns != "" && ns.front() == '/') {
    ns.erase(0, 1); // Remove the leading slash if present
  }

  // main loop
  while (ros::ok())
  {
    
    // create and fill message
    sensor_msgs::JointState msg;
    msg.name = {ns + "/gripper"};
    msg.position = {gripper_angle};
    
    

    geometry_msgs::Pose offset;
    offset.position.x = actuation_location[0];
    offset.position.y = actuation_location[1];
    offset.position.z = actuation_location[2];
    offset.orientation.x = 0;
    offset.orientation.y = 0;
    offset.orientation.z = 0;
    offset.orientation.w = 1;
  

    // publish message
    joint_pub.publish(msg);
    offset_pub.publish(offset);

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}

