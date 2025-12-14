#include <math.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "stdio.h"
#include "stdlib.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include <eigen3/Eigen/Dense>

# define M_PI           3.14159265358979323846

using namespace std;
using namespace Eigen;

/* ############################################### OVERVIEW ##########################################
 this node combines all the joint states from each arm and drivebase into a single message

*/
// ############################################### GLOBAL VARIABLES ##################################
sensor_msgs::JointState stickbug_joints;


// ############################################### FUNCTIONS #########################################
void add_joints(const sensor_msgs::JointState::ConstPtr& msg, int st_ind,int num_joints){
  // store the wrist joint names and positons
  ROS_INFO("start index %d, num joints %d", st_ind,num_joints);
 for(int i = 0; i < num_joints; i++){
    if(!isnan(msg->position[i])){
        stickbug_joints.position[i+st_ind] = msg->position[i];
    }
 }

}

//--------------------------------------------------- JOINT CALLBACK FUNCTIONS ----------------------
void drive_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,0,3);
}
void arm1_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,3,8);
}
void arm2_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,11,8);
}
void arm3_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,19,8);
}
void arm4_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,27,8);
}
void arm5_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,35,8);
}
void arm6_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  add_joints(msg,43,8);
}

// specify joint names and initialize positions to nan-----------------------------
void initialize_variables(){
    double nan = std::numeric_limits<double>::quiet_NaN();

    stickbug_joints.name = {"stickbug/drivebase/flw", "stickbug/drivebase/frw","stickbug/drivebase/bw",
                            "Nstickbug/arm1/slider", "Nstickbug/arm1/shoulder", "Nstickbug/arm1/elbow","Nstickbug/arm1/w1","Nstickbug/arm1/w2","Nstickbug/arm1/e1","Nstickbug/arm1/e2","Nstickbug/arm1/e3",
                            "stickbug/arm2/slider", "stickbug/arm2/shoulder", "stickbug/arm2/elbow","stickbug/arm2/w1","stickbug/arm2/w2","stickbug/arm2/e1","stickbug/arm2/e2","stickbug/arm2/e3",
                            "stickbug/arm3/slider", "stickbug/arm3/shoulder", "stickbug/arm3/elbow","stickbug/arm3/w1","stickbug/arm3/w2","stickbug/arm3/e1","stickbug/arm3/e2","stickbug/arm3/e3",
                            "stickbug/arm4/slider", "stickbug/arm4/shoulder", "stickbug/arm4/elbow","stickbug/arm4/w1","stickbug/arm4/w2","stickbug/arm4/e1","stickbug/arm4/e2","stickbug/arm4/e3",
                            "stickbug/arm5/slider", "stickbug/arm5/shoulder", "stickbug/arm5/elbow","stickbug/arm5/w1","stickbug/arm5/w2","stickbug/arm5/e1","stickbug/arm5/e2","stickbug/arm5/e3",
                            "stickbug/arm6/slider", "stickbug/arm6/shoulder", "stickbug/arm6/elbow","stickbug/arm6/w1","stickbug/arm6/w2","stickbug/arm6/e1","stickbug/arm6/e2","stickbug/arm6/e3"};
   stickbug_joints.position = {0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0};
}

//######################################## MAIN FUNCTION ########################################
int main(int argc, char **argv)
{
  
  initialize_variables();
  
  // ros node stuff
  ros::init(argc, argv, "stickbug_joint_pub_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  
  // publisher and subscribers
  ros::Publisher stickbug_joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Subscriber drive_joints = n.subscribe("drivebase/drivebase_joints", 10, drive_Callback);
  //ros::Subscriber arm1_joints = n.subscribe("arm1/manipulator_actual_joints", 10, arm1_Callback);
  ros::Subscriber arm2_joints = n.subscribe("arm2/manipulator_actual_joints", 10, arm2_Callback);
  ros::Subscriber arm3_joints = n.subscribe("arm3/manipulator_actual_joints", 10, arm3_Callback);
  ros::Subscriber arm4_joints = n.subscribe("arm4/manipulator_actual_joints", 10, arm4_Callback);
  ros::Subscriber arm5_joints = n.subscribe("arm5/manipulator_actual_joints", 10, arm5_Callback);
  ros::Subscriber arm6_joints = n.subscribe("arm6/manipulator_actual_joints", 10, arm6_Callback);

  // main loop--------------------------------------------

  while (ros::ok())
  {
    // publish messages
    stickbug_joints.header.stamp = ros::Time::now();
    stickbug_joint_pub.publish(stickbug_joints);
    
    // ros continue
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
