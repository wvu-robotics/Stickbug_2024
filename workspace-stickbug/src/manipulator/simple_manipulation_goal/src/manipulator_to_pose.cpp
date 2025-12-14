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

/*################################################################### OVERVIEW #####################
    
  This node fuses the ARM, WRIST, and EE inverse kinematics to manipulator inverse kinematics
  it itterates through poses and offsets for each part of the arm until it is below a tolarance

    main input is "manipulator_goal" geometry_msgs::Pose
    main output is "manipulator_goal_joints" sensor_msgs::Joint_State
    
*/
//#################################################### GLOBAL VARIABLES ############################

tf::Quaternion goal_orientation; // goal quaternion
VectorXd goal_pos(3);            // goal 3x1 position vector
bool got_goal = false;           // flag to see if a goal was published
double goal_tolarance = 1e-3;    // percentage of how close the target needs to be to the goal

VectorXd ee_offset(3);           // position offset of end effector
tf::Quaternion wrist_offset;     // orientation offset of the wrist due to the arm
VectorXd arm_offset(3);          // position offset of the arm due to the wrist

VectorXd arm_cumulative_offset(3);  // cumulative offset of the arm due to the wrist


sensor_msgs::JointState arm_joints;    // current arm goal joints
sensor_msgs::JointState wrist_joints;  // current wrist goal joints
sensor_msgs::JointState ee_joints;     // current end effector goal joints
int num_wrist_joints = 2;              // number of joints in the wrist (default, change with rosparam)
int num_arm_joints = 3;                // number of joints in the arm   (default, change with rosparam)
int num_ee_joints = 1;                 // number of joints in the end effector (default, change with rosparam)

double arm_pose_improvement = 100000;    // current arm goal change from previous target to current target
double wrist_pose_improvement = 100000;  // current wrist goal change from previous target to current target
double prev_improvement = 100000;
 
geometry_msgs::Pose wrist_goal;              // wrist goal position message
geometry_msgs::Pose arm_goal;                // arm goal position message
sensor_msgs::JointState manipulator_joints;  // manipulator goal joint state message

bool got_arm_joints = true;
bool got_wrist_joints = true;
bool got_ee_joints = true;

bool got_arm_offset = true;
bool got_wrist_offset = true;
bool got_ee_offset = true;

 

//################################# CALLBACK FUNCTIONS ################################################

// manipulator target goal input callback ---------------------------
void goal_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
   // convert goal position to Eigen library vector
   goal_pos(0) = msg->position.x;
   goal_pos(1) = msg->position.y;
   goal_pos(2) = msg->position.z;

   // pull the orientation quaternion out of the goal pose 
   tf::quaternionMsgToTF(msg->orientation, goal_orientation);

   //("i got a goal at x= %f y= %f z= %f ", goal_pos(0),goal_pos(1),goal_pos(2));

   got_goal = true;
  // ROS_INFO("I got a goal at x= %f y= %f z= %f ", goal_pos(0),goal_pos(1),goal_pos(2));
  
}

// end effector offset parameter callback ---------------------------
void ee_offset_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
   // pull the dx, dy, dz end effector offset out of the message
  ee_offset(0) = msg->position.x;
  ee_offset(1) = msg->position.y;
  ee_offset(2) = msg->position.z;
  got_ee_offset = true;
  
}

//  offset of the goal due to the wrist---------------------------------
void wrist_offset_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  // pull the orientation from the msg and store it as a quaternion
  tf::quaternionMsgToTF(msg->orientation, wrist_offset);
  got_wrist_offset = true;
  //ROS_INFO("I got an wrist offset");
}

// offset of the goal due to the arm ----------------------------------
void arm_offset_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  // pull out the dx, dy, dz of the arm goal
  arm_offset(0) = msg->position.x;
  arm_offset(1) = msg->position.y;
  arm_offset(2) = msg->position.z;
  got_arm_offset = true;
  //ROS_INFO("I got an arm offset x= %f y=%f z = %f", arm_offset(0),arm_offset(1),arm_offset(2));
}

// wrist joint angles---------------------------------------------------------
void wrist_joints_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{

  // store the wrist joint names and positons
 for(int i = 0; i < num_wrist_joints; i++){
    if(!isnan(msg->position[i])){
    	wrist_joints.name[i] = msg->name[i];
        wrist_joints.position[i] = msg->position[i];
    }
 }
 got_wrist_joints = true;
}

//  arm joint angles ---------------------------------------------------
void arm_joints_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  
  // store the arm joint names and positons
  for(int i = 0; i < num_arm_joints; i++){
      if(!isnan(msg->position[i])){
         arm_joints.name[i] = msg->name[i];
         arm_joints.position[i] = msg->position[i];
      }
  }
  got_arm_joints = true;
}

//  end effector joint angles ---------------------------------------------------
void ee_joints_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  
  // store the end effector joint names and positons
  for(int i = 0; i < num_ee_joints; i++){
      if(!isnan(msg->position[i])){
         ee_joints.name[i] = msg->name[i];
         ee_joints.position[i] = msg->position[i];
      }
  }
  got_ee_joints = true;
  got_goal = true;
}
// determine wrist goal improvement and goal with current offset-------------------------------------------------
void calculate_wrist_goal()
{
  tf::Quaternion adjusted_orientation = (goal_orientation*wrist_offset).normalize();
 
  // calculate how much the goal position of the wrist has changed 
  wrist_pose_improvement = abs(wrist_goal.position.x - ee_offset(0) +
                               wrist_goal.position.y - ee_offset(1) +
                               wrist_goal.position.z - ee_offset(2) +
                               wrist_goal.orientation.x - adjusted_orientation.x() +
                               wrist_goal.orientation.y - adjusted_orientation.y() +
                               wrist_goal.orientation.z - adjusted_orientation.z() +
                               wrist_goal.orientation.w - adjusted_orientation.w()  )/7;
                                    
  // fill the wrist goal with the new goal
  wrist_goal.position.x = ee_offset(0);
  wrist_goal.position.y = ee_offset(1);
  wrist_goal.position.z = ee_offset(2);
  wrist_goal.orientation.x = adjusted_orientation.x();
  wrist_goal.orientation.y = adjusted_orientation.y();
  wrist_goal.orientation.z = adjusted_orientation.z();
  wrist_goal.orientation.w = adjusted_orientation.w();
  
}
// determine arm goal improvement and goal with current offset -----------------------------------------------
void calcualte_arm_goal()
{
  // calculate how much the goal position of the arm has changed
  arm_pose_improvement = abs(arm_goal.position.x - (goal_pos(0) + arm_offset(0)) +
                             arm_goal.position.y - (goal_pos(1) + arm_offset(1)) +
                             arm_goal.position.z - (goal_pos(2) + arm_offset(2))  )/3;
  

  //need to rotate the arm offset to be the base arm frame from wrist frame by wrist_offset
  //Convert Eigen Vector to tf::Vector3
  tf::Vector3 arm_offset_tf(arm_offset(0), arm_offset(1), arm_offset(2));

  // Convert the quaternion to a rotation matrix
  tf::Matrix3x3 rotation_matrix(wrist_offset);
  // flip the rotation matrix
  rotation_matrix = rotation_matrix.transpose();

  // Rotate the vector by the quaternion
  tf::Vector3 rotated_arm_offset = rotation_matrix * arm_offset_tf;
  //ROS_INFO("rotated arm offset x = %f y = %f z = %f", rotated_arm_offset.x(), rotated_arm_offset.y(), rotated_arm_offset.z());

  arm_cumulative_offset(0) = arm_cumulative_offset(0) + .1*(rotated_arm_offset.x() - arm_cumulative_offset(0));
  arm_cumulative_offset(1) = arm_cumulative_offset(1) + .1*(rotated_arm_offset.y() - arm_cumulative_offset(1));
  arm_cumulative_offset(2) = arm_cumulative_offset(2) + .1*(rotated_arm_offset.z() - arm_cumulative_offset(2));

  // Fill the arm goal with the new goal
  arm_goal.position.x = goal_pos(0) + arm_cumulative_offset(0); //- rotated_arm_offset.y();
  arm_goal.position.y = goal_pos(1) + arm_cumulative_offset(1);//+ rotated_arm_offset.x();
  arm_goal.position.z = goal_pos(2) + arm_cumulative_offset(2);//+ rotated_arm_offset.z();
}

// fill goal manipulator joints from the arm and wrist joints----------------------------
void fill_manipulator_joints()
{   
   manipulator_joints.name = {};
   manipulator_joints.position = {};
   manipulator_joints.velocity = {};
   manipulator_joints.header.stamp = ros::Time::now();


  if(num_arm_joints > 0 ){
    // add arm joints
    for(int i = 0; i < num_arm_joints; i++)
    {
      manipulator_joints.name.push_back(arm_joints.name[i]);
      manipulator_joints.position.push_back(arm_joints.position[i]);
      manipulator_joints.velocity.push_back(0.0);
    }
  }
  if(num_wrist_joints > 0){
    // add wrist joints
    for(int i = 0; i < num_wrist_joints; i++)
    {
      manipulator_joints.name.push_back(wrist_joints.name[i]);
      manipulator_joints.position.push_back(wrist_joints.position[i]);
      manipulator_joints.velocity.push_back(0.0);
    }

  }
  if(num_ee_joints > 0){
    // add end effector joints
    for(int i = 0; i < num_ee_joints; i++)
    {
      manipulator_joints.name.push_back(ee_joints.name[i]);
      manipulator_joints.position.push_back(ee_joints.position[i]);
      manipulator_joints.velocity.push_back(0.0);
    }

  }

}

//  initialize msg variables ------------------------------------------------------
void initialize_variables(string ns){
    ee_offset(0) = 0.0;
    ee_offset(1) = 0.0;
    ee_offset(2) = 0.0;

    arm_offset(0) = 0;
    arm_offset(1) = 0;
    arm_offset(2) = 0;

    wrist_offset = tf::Quaternion(0,0,0,1);

    if (ns != "" && ns.front() == '/') {
      ns.erase(0, 1); // Remove the leading slash if present
    }

    arm_joints.name =  {ns +"/slider", ns +"/shoulder", ns +"/elbow"};
    arm_joints.position = {.5,1.57,1.57};

    wrist_joints.name = {ns +"/w1",ns +"/w2"};
    wrist_joints.position = {0.0,0.0};

    ee_joints.name = {ns +"/e1",ns +"/e2",ns +"/e3"};
    ee_joints.position = {0.0, 0.0, 0.0};

    arm_cumulative_offset(0) = 0;
    arm_cumulative_offset(1) = 0;
    arm_cumulative_offset(2) = 0;

}
//################################ MAIN FUNCTION ################################################
int main(int argc, char **argv)
{
  
  // initalize msg variables
  
  ROS_INFO("initialized variables");
  
  // ros node initalization
  ros::init(argc, argv, "manipulator_to_pose_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100); //100
  std::string ns = ros::this_node::getNamespace();

  initialize_variables(ns);

  // ros parameters
  if(ros::param::get(ns + "/num_arm_joints",num_arm_joints)==false)
		ROS_WARN("No parameter %s/num_arm_joints specified, using default 3", ns.c_str());

  if(ros::param::get(ns + "/num_wrist_joints",num_wrist_joints)==false)
		ROS_WARN("No parameter %s/num_wrist_joints specified, using default 2", ns.c_str());

  if(ros::param::get(ns + "/num_ee_joints",num_ee_joints)==false)
		ROS_WARN("No parameter %s/num_ee_joints specified, using default 1", ns.c_str());    
  
  // publisher and subscribers
  ros::Publisher manipulator_joint_pub = n.advertise<sensor_msgs::JointState>("manipulator_goal_joints", 10);
  ros::Publisher wrist_goal_pub = n.advertise<geometry_msgs::Pose>("wrist_goal",10);
  ros::Publisher arm_goal_pub = n.advertise<geometry_msgs::Pose>("arm_goal",10);
  
  ros::Subscriber manipulator_goal_sub = n.subscribe("manipulator_goal", 10, goal_Callback);
  ros::Subscriber ee_offset_sub = n.subscribe("ee_offset", 10, ee_offset_Callback);
  ros::Subscriber wrist_offset_sub = n.subscribe("wrist_offset", 10, wrist_offset_Callback);
  ros::Subscriber arm_offset_sub = n.subscribe("arm_offset", 10, arm_offset_Callback);
  ros::Subscriber wrist_joints_sub = n.subscribe("wrist_joints", 10, wrist_joints_Callback);
  ros::Subscriber arm_joints_sub = n.subscribe("arm_joints", 10, arm_joints_Callback);
  ros::Subscriber ee_joints_sub = n.subscribe("ee_joints", 10, ee_joints_Callback);

  ROS_INFO("made subscribers and publishers");

  // main loop--------------------------------------------
  double count = 0;
  while (ros::ok())
  {
    
    // if a goal was recieved calculate IK and publish joints
    if(got_goal){
      //calculate the improvement of the Inverse Kinematics
      double total_improvement = abs(arm_pose_improvement + wrist_pose_improvement);

      // if good enough, publish the manipulator goal joint states
     if((total_improvement < goal_tolarance && prev_improvement < goal_tolarance) || count > 100){
       ROS_INFO("published manipulator goal joints");
       fill_manipulator_joints();
       manipulator_joint_pub.publish(manipulator_joints);
       got_goal = false;
       arm_pose_improvement = 100000;    
       wrist_pose_improvement = 100000;
       prev_improvement = 100000;
       count = 0;
     }
     else{// recalculate and publish messages to the arm and wrist
       if(got_arm_joints && got_arm_offset){
        calculate_wrist_goal();
        got_arm_joints = false;
        got_arm_offset = false;
        wrist_goal_pub.publish(wrist_goal);
        count++;
       }

       if(got_wrist_joints && got_wrist_offset){
        calcualte_arm_goal();
        got_wrist_joints = false;
        got_wrist_offset = false;
        arm_goal_pub.publish(arm_goal);
        count++;
       }
       

       //ROS_INFO("%f the current total improvement is %f", count,total_improvement);
      }
      prev_improvement = total_improvement;
      
    } 

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}










