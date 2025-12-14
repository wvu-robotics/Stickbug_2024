#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>

#include "stdio.h"
#include "stdlib.h"
#include <unistd.h>  

# define M_PI           3.14159265358979323846

using namespace std;

/*###################################################################### Overview #####################################################################################
 *                             
 *  Description: This node is used for general control over the pollination end effector    
 *  
 *  NOTE: it is ASSUMED that all closed loop control of the end effector is housed on an arduino or sub-computing device 
 * 
 *=====================================================================================================================================================================
 *                 TOP VIEW                           |           OVERVIEW
 *                                                    |
 *                                                    |   1. subscribes to <namespace>/end_effector_on std_msgs::Boolean    
 *                     x                              |   2. publishes  to <namespace>/ee_offset  std_msgs::Pose
 *     ^  X            O                              |   3. publishes  to <namespace>/ee_joints  sensor_msgs::JointState
 * Y   |              /|\                             |   
 * <---0             / | \                            |____________________________________________________________________
 *      Z           /  |  \                           |           FRONT VIEW
 *                (1) (2) (3)                         |            (2)
 *                 +---+---+--- motor joint           |             |
 *                                                    |             O    
 *    NOTE: 'x' is the actuation location             |            / \
 *                                                    |         (3)   (1)
*///################################################################## Global Variables #################################################################################

// ros1 bridge https://github.com/ros2/ros1_bridge/blob/master/README.md

double actuation_x = .02; // the dx position from the end effector base to the desired actuation location on the gripper
double actuation_y = 0;   // the dy position from the end effector base to the desired actuation location on the gripper
double actuation_z = 0;   // the dz position from the end effector base to the desired actuation location on the gripper

double e1 = 0.0;                          // end effector motor 1's position (right from front) [m]
double e2 = 0.0;                          // end effector motor 2's position (top from front)   [m]
double e3 = 0.0;                          // end effector motor 3's position (left from front)  [m]

double max_position = .1;                // max position of the end effector motor [m]
double min_position = 0.0;                // min position of the end effector motor [m]

int sequence_num = 0;
bool got_manipulator_joints = false;

bool start_pollination = false;           // starts pollination sequence

//################################################################# Command Callback ##################################################################################

void eeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    start_pollination = msg->data;
}

void manCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    got_manipulator_joints = true;
}

//################################################################## MAIN #################################################################################

int main(int argc, char **argv)
{

  // ros node stuff
  ros::init(argc, argv, "pollination_ee_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

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
   if(ros::param::get(ns + "/max_position",max_position)==false)
	{
		ROS_WARN("No parameter %s/max_position specified, USING DEFAULT %f",ns.c_str(),max_position);
	}
  if(ros::param::get(ns + "/min_position",min_position)==false)
	{
		ROS_WARN("No parameter %s/min_position specified, USING DEFAULT %f",ns.c_str(),min_position);
	}
  // initalization ---------------------------------------------------------------------------------------------------
  double actuation_location[] = {actuation_x,actuation_y,actuation_z}; // houses the [dx,dy,dz] position from the end effector base to the desired actuation location on the gripper
  
  
  // publisher and subscribers
  ros::Publisher offset_pub = n.advertise<geometry_msgs::Pose>("ee_offset", 10);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("ee_joints", 10);
  ros::Publisher done_pub = n.advertise<std_msgs::Bool>("ee_done", 10);
  ros::Subscriber ee_sub = n.subscribe("end_effector_on", 10, eeCallback);
  ros::Subscriber man_sub = n.subscribe("manipulator_goal_joints", 10, manCallback);

  if (ns != "" && ns.front() == '/') {
    ns.erase(0, 1); // Remove the leading slash if present
  }

  // main loop
  bool first = true;
  while (ros::ok())
  {
    
    // create and fill message
    sensor_msgs::JointState msg;
    msg.name = {ns + "/e1", ns + "/e2", ns + "/e3"};
    msg.position = {e1,e2,e2};
    
    if(first){
      joint_pub.publish(msg);
      first = false;
    }

    
    geometry_msgs::Pose offset;
    offset.position.x = actuation_location[0];
    offset.position.y = actuation_location[1];
    offset.position.z = actuation_location[2];
    offset.orientation.x = 0;
    offset.orientation.y = 0;
    offset.orientation.z = 0;
    offset.orientation.w = 1;

    offset_pub.publish(offset);

    // pollination sequence
    if(start_pollination && got_manipulator_joints){
        
        // position 1
        if (sequence_num == 0){
          sleep( 1 ); 
          e1 = max_position;
          e2 = min_position;
          e3 = min_position;
          msg.position = {e1,e2,e3};
          joint_pub.publish(msg);
          std_msgs::Bool done_msg;
          done_msg.data = false;
          done_pub.publish(done_msg);
          sequence_num = 1;
          ROS_INFO("started pollination");
          got_manipulator_joints = false;
        }

        // position 2
        else if (sequence_num == 1 && got_manipulator_joints){
          sleep( 1 ); 
          e1 = min_position;
          e2 = max_position;
          e3 = min_position;
          msg.position = {e1,e2,e3};
          joint_pub.publish(msg);
          std_msgs::Bool done_msg;
          done_msg.data = false;
          done_pub.publish(done_msg);
          sequence_num = 2;
          got_manipulator_joints = false;
        }

        // position 3
        else if (sequence_num == 2 && got_manipulator_joints){
          sleep( 1 ); 
          e1 = min_position;
          e2 = min_position;
          e3 = max_position;
          msg.position = {e1,e2,e3};
          joint_pub.publish(msg);
          std_msgs::Bool done_msg;
          done_msg.data = false;
          done_pub.publish(done_msg);
          sequence_num = 3;
          got_manipulator_joints = false;
        }

        // end sequence
        else if (sequence_num == 3 && got_manipulator_joints){
          sleep( 1 ); 
          e1 = min_position;
          e2 = min_position;
          e3 = min_position;
          msg.position = {e1,e2,e3};
          // publish message
          joint_pub.publish(msg);

          std_msgs::Bool done_msg;
          done_msg.data = true;
          done_pub.publish(done_msg);
          start_pollination = false;
          got_manipulator_joints = false;
          sequence_num = 0;
        }
    }
  
   

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}

