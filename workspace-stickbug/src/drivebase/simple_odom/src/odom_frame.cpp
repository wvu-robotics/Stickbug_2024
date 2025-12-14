#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>

// rosparam macro
#define SAFE_GET_PARAM(name, var) \
{\
	if (pn.hasParam(name)){\
		pn.getParam(name, var);\
	} else {\
		ROS_WARN("Could not locate %s, using default: %s", name, var.c_str());\
	}\
}

/*################################################ OVERVIEW ###############################################################
* 
*               frame setup                         |      1. recieves the odom topic from the robot localization
*                                                   |      2. coverts the odomm topic to a TF and broadcasts it
*                     RX                            |      
*                     ^                             |
*                     |  Yaw                        |    
*      ^ Y     RY <--[^] ----> X                    |      
*      |         robot_frame "base_footprint"       |        
*      |                                            |
*      O----> X                                     |
*    Z                                              |
*   odometry_frame "odom"                           |
*                                                   |
*===========================================================================================================================
*
*  subscribes to "odom" of type geometry_msgs::Twist 
*  broadcasts "odom" to "base_footprint" TF  
*
*///======================================================== GLOBAL VARIABLES ====================================================================
nav_msgs::Odometry odom;  //odometry of the robot

//========================================================= CALLBACK FUNCTION ==============================================================
void callback(const nav_msgs::Odometry &msg){
  odom.pose = msg.pose;
  odom.twist = msg.twist;
}

//========================================================= MAIN FUNCTION ==================================================================
int main(int argc, char** argv){
  
  // setup ros node
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
	ros::NodeHandle pn("~");
  ros::Rate r(1.0);

 // get parameters for the head frame, child framem, and topic odom is on
	std::string head = "/odom", child = "/base_footprint", top = "odom";
	SAFE_GET_PARAM("head", head);
	SAFE_GET_PARAM("child", child);
	SAFE_GET_PARAM("topic", top);
  
  // set subscriber and tf broadcaster
  tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber odom_sub = n.subscribe(top, 1000, callback);
 
  // setup current and previous time
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

 // write out what topic names and frame names it is using
	ROS_INFO("Looping on %s: creating %s->%s", top.c_str(), head.c_str(), child.c_str());

  // initiallize the odometry to zero
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

 // main loop
  while(n.ok()){
    
    // check for incoming messages
    ros::spinOnce();               
    current_time = ros::Time::now();


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = head;
    odom_trans.child_frame_id = child;

    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    odom_trans.transform.rotation = odom.pose.pose.orientation;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    last_time = current_time;
    r.sleep();
  }
}
