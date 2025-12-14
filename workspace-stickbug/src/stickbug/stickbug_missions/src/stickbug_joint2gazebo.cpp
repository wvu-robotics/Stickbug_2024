#include <math.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
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
    double nan = 0; //std::numeric_limits<double>::quiet_NaN();

    stickbug_joints.name = {"flw", "frw","bw",
                            "slider_1", "shoulder_1", "elbow_1","w1_1","w2_1","e1_1","e2_1","e3_1",
                            "slider_2", "shoulder_2", "elbow_2","w1_2","w2_2","e1_2","e2_2","e3_2",
                            "slider_3", "shoulder_3", "elbow_3","w1_3","w2_3","e1_3","e2_3","e3_3",
                            "slider_4", "shoulder_4", "elbow_4","w1_4","w2_4","e1_4","e2_4","e3_4",
                            "slider_5", "shoulder_5", "elbow_5","w1_5","w2_5","e1_5","e2_5","e3_5",
                            "slider_6", "shoulder_6", "elbow_6","w1_6","w2_6","e1_6","e2_6","e3_6",};
   stickbug_joints.position = {nan,nan,nan,
                               nan,nan,nan,nan,nan,nan,nan,nan,
                               nan,nan,nan,nan,nan,nan,nan,nan,
                               nan,nan,nan,nan,nan,nan,nan,nan,
                               nan,nan,nan,nan,nan,nan,nan,nan,
                               nan,nan,nan,nan,nan,nan,nan,nan,
                               nan,nan,nan,nan,nan,nan,nan,nan};
}


//######################################## MAIN FUNCTION ########################################
int main(int argc, char **argv)
{
  
  initialize_variables();
  
  // ros node stuff
  ros::init(argc, argv, "stickbug_joint_pub_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  // publisher and subscribers
  
  // drive train motor publishers
  ros::Publisher bw_pub  = n.advertise<std_msgs::Float64>("bw_position_controller/command",  10);
  ros::Publisher frw_pub = n.advertise<std_msgs::Float64>("frw_position_controller/command", 10);
  ros::Publisher flw_pub = n.advertise<std_msgs::Float64>("flw_position_controller/command", 10);

  // arm 1 motor publishers 
  ros::Publisher slider_1_pub  = n.advertise<std_msgs::Float64>("slider_1_position_controller/command",  10);
  ros::Publisher shoulder_1_pub = n.advertise<std_msgs::Float64>("shoulder_1_position_controller/command", 10);
  ros::Publisher elbow_1_pub = n.advertise<std_msgs::Float64>("elbow_1_position_controller/command", 10);
  ros::Publisher w1_1_pub = n.advertise<std_msgs::Float64>("w1_1_position_controller/command",  10);
  ros::Publisher w2_1_pub = n.advertise<std_msgs::Float64>("w2_1_position_controller/command", 10);
  ros::Publisher e1_1_pub = n.advertise<std_msgs::Float64>("e1_1_position_controller/command", 10);
  ros::Publisher e2_1_pub = n.advertise<std_msgs::Float64>("e2_1_position_controller/command", 10);
  ros::Publisher e3_1_pub = n.advertise<std_msgs::Float64>("e3_1_position_controller/command", 10);
  
  // arm 2 motor publishers
  ros::Publisher slider_2_pub  = n.advertise<std_msgs::Float64>("slider_2_position_controller/command",  10);
  ros::Publisher shoulder_2_pub = n.advertise<std_msgs::Float64>("shoulder_2_position_controller/command", 10);
  ros::Publisher elbow_2_pub = n.advertise<std_msgs::Float64>("elbow_2_position_controller/command", 10);
  ros::Publisher w1_2_pub = n.advertise<std_msgs::Float64>("w1_2_position_controller/command",  10);
  ros::Publisher w2_2_pub = n.advertise<std_msgs::Float64>("w2_2_position_controller/command", 10);
  ros::Publisher e1_2_pub = n.advertise<std_msgs::Float64>("e1_2_position_controller/command", 10);
  ros::Publisher e2_2_pub = n.advertise<std_msgs::Float64>("e2_2_position_controller/command", 10);
  ros::Publisher e3_2_pub = n.advertise<std_msgs::Float64>("e3_2_position_controller/command", 10);

  // arm 3 motor publishers
  ros::Publisher slider_3_pub  = n.advertise<std_msgs::Float64>("slider_3_position_controller/command",  10);
  ros::Publisher shoulder_3_pub = n.advertise<std_msgs::Float64>("shoulder_3_position_controller/command", 10);
  ros::Publisher elbow_3_pub = n.advertise<std_msgs::Float64>("elbow_3_position_controller/command", 10);
  ros::Publisher w1_3_pub = n.advertise<std_msgs::Float64>("w1_3_position_controller/command",  10);
  ros::Publisher w2_3_pub = n.advertise<std_msgs::Float64>("w2_3_position_controller/command", 10);
  ros::Publisher e1_3_pub = n.advertise<std_msgs::Float64>("e1_3_position_controller/command", 10);
  ros::Publisher e2_3_pub = n.advertise<std_msgs::Float64>("e2_3_position_controller/command", 10);
  ros::Publisher e3_3_pub = n.advertise<std_msgs::Float64>("e3_3_position_controller/command", 10);

  // arm 4 motor publishers
  ros::Publisher slider_4_pub  = n.advertise<std_msgs::Float64>("slider_4_position_controller/command",  10);
  ros::Publisher shoulder_4_pub = n.advertise<std_msgs::Float64>("shoulder_4_position_controller/command", 10);
  ros::Publisher elbow_4_pub = n.advertise<std_msgs::Float64>("elbow_4_position_controller/command", 10);
  ros::Publisher w1_4_pub = n.advertise<std_msgs::Float64>("w1_4_position_controller/command",  10);
  ros::Publisher w2_4_pub = n.advertise<std_msgs::Float64>("w2_4_position_controller/command", 10);
  ros::Publisher e1_4_pub = n.advertise<std_msgs::Float64>("e1_4_position_controller/command", 10);
  ros::Publisher e2_4_pub = n.advertise<std_msgs::Float64>("e2_4_position_controller/command", 10);
  ros::Publisher e3_4_pub = n.advertise<std_msgs::Float64>("e3_4_position_controller/command", 10);

  // arm 5 motor publishers
  ros::Publisher slider_5_pub  = n.advertise<std_msgs::Float64>("slider_5_position_controller/command",  10);
  ros::Publisher shoulder_5_pub = n.advertise<std_msgs::Float64>("shoulder_5_position_controller/command", 10);
  ros::Publisher elbow_5_pub = n.advertise<std_msgs::Float64>("elbow_5_position_controller/command", 10);
  ros::Publisher w1_5_pub = n.advertise<std_msgs::Float64>("w1_5_position_controller/command",  10);
  ros::Publisher w2_5_pub = n.advertise<std_msgs::Float64>("w2_5_position_controller/command", 10);
  ros::Publisher e1_5_pub = n.advertise<std_msgs::Float64>("e1_5_position_controller/command", 10);
  ros::Publisher e2_5_pub = n.advertise<std_msgs::Float64>("e2_5_position_controller/command", 10);
  ros::Publisher e3_5_pub = n.advertise<std_msgs::Float64>("e3_5_position_controller/command", 10);

  // arm 6 motor publishers
  ros::Publisher slider_6_pub  = n.advertise<std_msgs::Float64>("slider_6_position_controller/command",  10);
  ros::Publisher shoulder_6_pub = n.advertise<std_msgs::Float64>("shoulder_6_position_controller/command", 10);
  ros::Publisher elbow_6_pub = n.advertise<std_msgs::Float64>("elbow_6_position_controller/command", 10);
  ros::Publisher w1_6_pub = n.advertise<std_msgs::Float64>("w1_6_position_controller/command",  10);
  ros::Publisher w2_6_pub = n.advertise<std_msgs::Float64>("w2_6_position_controller/command", 10);
  ros::Publisher e1_6_pub = n.advertise<std_msgs::Float64>("e1_6_position_controller/command", 10);
  ros::Publisher e2_6_pub = n.advertise<std_msgs::Float64>("e2_6_position_controller/command", 10);
  ros::Publisher e3_6_pub = n.advertise<std_msgs::Float64>("e3_6_position_controller/command", 10);

  // joint subscribers 
  ros::Subscriber drive_joints = n.subscribe("drivebase/drivebase_joints", 10, drive_Callback);
  ros::Subscriber arm1_joints = n.subscribe("arm1/manipulator_goal_joints", 10, arm1_Callback);
  ros::Subscriber arm2_joints = n.subscribe("arm2/manipulator_goal_joints", 10, arm2_Callback);
  ros::Subscriber arm3_joints = n.subscribe("arm3/manipulator_goal_joints", 10, arm3_Callback);
  ros::Subscriber arm4_joints = n.subscribe("arm4/manipulator_goal_joints", 10, arm4_Callback);
  ros::Subscriber arm5_joints = n.subscribe("arm5/manipulator_goal_joints", 10, arm5_Callback);
  ros::Subscriber arm6_joints = n.subscribe("arm6/manipulator_goal_joints", 10, arm6_Callback);

  // main loop--------------------------------------------

  while (ros::ok())
  {
    
    std_msgs::Float64 msg;

    // publish drive joints to gazebo
    msg.data = stickbug_joints.position[0];
    flw_pub.publish(msg);
    msg.data = stickbug_joints.position[1];
    frw_pub.publish(msg);
    msg.data = stickbug_joints.position[2];
    bw_pub.publish(msg);
    
    // publish arm 1 joints to gazebo
    msg.data = stickbug_joints.position[3];
    slider_1_pub.publish(msg);
    msg.data = stickbug_joints.position[4];
    shoulder_1_pub.publish(msg);
    msg.data = stickbug_joints.position[5];
    elbow_1_pub.publish(msg);
    msg.data = stickbug_joints.position[6];
    w1_1_pub.publish(msg);
    msg.data = stickbug_joints.position[7];
    w2_1_pub.publish(msg);
    msg.data = stickbug_joints.position[8];
    e1_1_pub.publish(msg);
    msg.data = stickbug_joints.position[9];
    e2_1_pub.publish(msg);
    msg.data = stickbug_joints.position[10];
    e3_1_pub.publish(msg);

    // publish arm 2 joints to gazebo
    msg.data = stickbug_joints.position[11];
    slider_2_pub.publish(msg);
    msg.data = stickbug_joints.position[12];
    shoulder_2_pub.publish(msg);
    msg.data = stickbug_joints.position[13];
    elbow_2_pub.publish(msg);
    msg.data = stickbug_joints.position[14];
    w1_2_pub.publish(msg);
    msg.data = stickbug_joints.position[15];
    w2_2_pub.publish(msg);
    msg.data = stickbug_joints.position[16];
    e1_2_pub.publish(msg);
    msg.data = stickbug_joints.position[17];
    e1_2_pub.publish(msg);
    msg.data = stickbug_joints.position[18];
    e2_2_pub.publish(msg);
    msg.data = stickbug_joints.position[19];
    e3_2_pub.publish(msg);


    // publish arm 3 joints to gazebo
    msg.data = stickbug_joints.position[20];
    slider_3_pub.publish(msg);
    msg.data = stickbug_joints.position[21];
    shoulder_3_pub.publish(msg);
    msg.data = stickbug_joints.position[22];
    elbow_3_pub.publish(msg);
    msg.data = stickbug_joints.position[23];
    w1_3_pub.publish(msg);
    msg.data = stickbug_joints.position[24];
    w2_3_pub.publish(msg);
    msg.data = stickbug_joints.position[25];
    e1_3_pub.publish(msg);
    msg.data = stickbug_joints.position[26];
    e2_3_pub.publish(msg);
    msg.data = stickbug_joints.position[27];
    e3_3_pub.publish(msg);

    // publish arm 4 joints to gazebo
    msg.data = stickbug_joints.position[28];
    slider_4_pub.publish(msg);
    msg.data = stickbug_joints.position[29];
    shoulder_4_pub.publish(msg);
    msg.data = stickbug_joints.position[30];
    elbow_4_pub.publish(msg);
    msg.data = stickbug_joints.position[31];
    w1_4_pub.publish(msg);
    msg.data = stickbug_joints.position[32];
    w2_4_pub.publish(msg);
    msg.data = stickbug_joints.position[33];
    e1_4_pub.publish(msg);
    msg.data = stickbug_joints.position[34];
    e2_4_pub.publish(msg);
    msg.data = stickbug_joints.position[35];
    e3_4_pub.publish(msg);
    

    // publish arm 5 joints to gazebo
    msg.data = stickbug_joints.position[36];
    slider_5_pub.publish(msg);
    msg.data = stickbug_joints.position[37];
    shoulder_5_pub.publish(msg);
    msg.data = stickbug_joints.position[38];
    elbow_5_pub.publish(msg);
    msg.data = stickbug_joints.position[39];
    w1_5_pub.publish(msg);
    msg.data = stickbug_joints.position[40];
    w2_5_pub.publish(msg);
    msg.data = stickbug_joints.position[41];
    e1_5_pub.publish(msg);
    msg.data = stickbug_joints.position[42];
    e2_5_pub.publish(msg);
    msg.data = stickbug_joints.position[43];
    e3_5_pub.publish(msg);

    // publish arm 6 joints to gazebo
    msg.data = stickbug_joints.position[44];
    slider_6_pub.publish(msg);
    msg.data = stickbug_joints.position[45];
    shoulder_6_pub.publish(msg);
    msg.data = stickbug_joints.position[46];
    elbow_6_pub.publish(msg);
    msg.data = stickbug_joints.position[47];
    w1_6_pub.publish(msg);
    msg.data = stickbug_joints.position[48];
    w2_6_pub.publish(msg);
    msg.data = stickbug_joints.position[49];
    e1_6_pub.publish(msg);
    msg.data = stickbug_joints.position[50];
    e2_6_pub.publish(msg);
    msg.data = stickbug_joints.position[51];
    e3_6_pub.publish(msg);
    
    // ros continue
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
