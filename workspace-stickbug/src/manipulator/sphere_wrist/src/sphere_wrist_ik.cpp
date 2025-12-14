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
/*#################################################################  OVERVIEW ################################################################################
*   Purpose: 
*          1. to solve the inverse kinematics of the spherical wrist 
*          2. determine how the arm goal should be offset for the desired goal location
*  
*   Requires: 
*             1. adjust link lengths to match the actual arm
*             2. the end effector offset to be published (or assumed zero)
*             3. a goal orientation for the wrist
*
*##############################################################################################################################################################
*                                                                          |
*                                                                          |  Kinematics are done using (4x4) homogenous transformation matricies
*                                                                          |   i.e) 
*    ZG ^           Z' ^            Z" ^                 ^ Z"'             | P (4x1) =  [Rotation (3x3)     displacement (3x1)] * P_old (4x1)
*       |              |               |                 |                 |            [    0 0 0                   1        ]      
*       |   XG         |    X'         |                 |                 |  
*       O----> ======= O-----> ==][== O----> X"'=][===== O----> X"'        |  1. motor 1 is the direct YAW motor
*   "base"           "motor 1"      "motor 2"          "motor 3"           |  2. motor 2 is the direct PITCH motor
*       |--- link 1 ---|                                                   |  3. Motor 3 is the direct ROLL motor
*                      |---= link 2 --|                                    |  
*                                     |---- link 3 ------|                 |  
*                                                                          |  
*                                                                          |  
*                                                                          |  
*                                                                  
*
*
*///##################################################################  Global Variables  ######################################################################

double roll;    // rotation about x axis  [rad]
double pitch;   // rotation about y axis  [rad]  --> come from goal quaternion orientation
double yaw;     // rotation about z axis  [rad]

VectorXd pose_vector(3);       // goal orientation unit vector

double m1 = 0;   // first wrist motor goal angle  [rad]
double m2 = 0;   // second wrist motor goal angle [rad]
double m3 = 0;   // last wrist motor goal angle   [rad]

VectorXd link1(3); // link connecting the base of the wrist to motor 1 axis, in base frame [dx, dy, dz] [m]
VectorXd link2(3); // link connecting the motor 1 axis to motor 2 axis, in motor 1 frame [dx, dy, dz] [m]
VectorXd link3(3); // link connecting the motor 2 axis to motor 3 axis, in motor 2 frame [dx, dy, dz] [m]

// VVV note VVV end_effector is a homogenous transformation vector that is why it has 4 indexes
VectorXd end_effector(4); // link connecting motor 3 to the tip of the end effector, in motor 3 frame [dx, dy, dz, 1] [m]

VectorXd arm_offset(3);   // the adjustment on the position of the arm due to the wrist having length [dx, dy, dz] [m]

bool got_goal = false;    // flag to see if a goal was recieved

//################################################################  Callback Functions  #############################################################################


//------------------------------------------------------------------------ goal pose callback 
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

 // pull the end effector offset out of the positon
  end_effector(0) = msg->position.x;
  end_effector(1) = msg->position.y;
  end_effector(2) = msg->position.z;
  end_effector(3) = 1;
 
 // pull the orientation quaternion out of the goal pose, and convert to RPY, and a rotation matrix 
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3 rm = tf::Matrix3x3(quat); 
  rm.getRPY(roll, pitch, yaw);

  // flag that we got our goal
  got_goal = true;
}

//------------------------------------------------------------------------------- nois wrist position inverse kinematics
void sphere_ik(){
    
   if (!std::isnan(yaw) && !std::isnan(pitch) && !std::isnan(roll)) {
    m1 = yaw;
    m2 = pitch;
    m3 = roll;
   }

}

//------------------------------------------------------------------------------- determine the arm offset 
void calc_arm_offset(){

    // use forward kinematics to determine how the wrist changes the end effectors location
    // the dx, dy, and dz is the offset from the goal to subtract off the arm goal's position
 
 // translation from base frame to motor 1 frame and rotate "m1" about Z
 MatrixXd Rz(4,4);
   Rz(0,0) = cos(m1);    Rz(0,1) = -sin(m1);   Rz(0,2) = 0;     Rz(0,3) = link1(0);
	Rz(1,0) = sin(m1);    Rz(1,1) = cos(m1);    Rz(1,2) = 0;     Rz(1,3) = link1(1);
	Rz(2,0) = 0;          Rz(2,1) = 0;          Rz(2,2) = 1;     Rz(2,3) = link1(2);
	Rz(3,0) = 0;          Rz(3,1) = 0;          Rz(3,2) = 0;     Rz(3,3) = 1;
 

 // translation from motor 1 frame to motor 2 frame and rotate "m2" about y  
 MatrixXd Ry(4,4);
   Ry(0,0) = cos(m2);       Ry(0,1) = 0;   Ry(0,2) = sin(m2);    Ry(0,3) = link2(0);
	Ry(1,0) = 0;             Ry(1,1) = 1;   Ry(1,2) = 0;           Ry(1,3) = link2(1);
	Ry(2,0) = -sin(m2);       Ry(2,1) = 0;   Ry(2,2) = cos(m2);     Ry(2,3) = link2(2);
	Ry(3,0) = 0;             Ry(3,1) = 0;   Ry(3,2) = 0;           Ry(3,3) = 1;

    	
 // translation from motor 2 frame to motor 3 frame and rotate "m3" about x  
  MatrixXd Rx(4,4);
   Rx(0,0) = 1;          Rx(0,1) = 0;           Rx(0,2) = 0;          Rx(0,3) = link3(0);
	Rx(1,0) = 0;          Rx(1,1) = cos(m3);     Rx(1,2) = -sin(m3);   Rx(1,3) = link3(1);
	Rx(2,0) = 0;          Rx(2,1) = sin(m3);     Rx(2,2) = cos(m3);    Rx(2,3) = link3(2);
	Rx(3,0) = 0;          Rx(3,1) = 0;           Rx(3,2) = 0;          Rx(3,3) = 1;

 // calculate forward kinematics by multiplying all the rotation matricies and the end effector offset
    VectorXd offset_point = Rz*Ry*Rx*end_effector;

   // convert back to global frame
   //pull the dx, dy, dz and negate to turn into the arm offset
   // i.e) "goal_pose" + "arm_offset" = "arm_goal"
    
    arm_offset(0) = -offset_point(0);
    arm_offset(1) = -offset_point(1);
    arm_offset(2) = -offset_point(2);
    
    //arm_offset(0) = -offset_point(2);
    //arm_offset(1) =  offset_point(1);
    //arm_offset(2) = -offset_point(0);
}

/*################################################################## MAIN #################################################################################
*
*   1. subscribes to "wrist_goal" of type geometry_msgs::Pose
*      extracts quaternion from msg and calculates motor joint angles
*      extracts x,y,z and sets it to be the change in position from 
*      the tip of the wrist to the tip of the end effector
*
*   2. publishes to "arm_offset" of type geometry_msgs::Pose
*           Geometry_msgs/Point position = [dx [m], dy [m]. dz [m]]      
*           Geometry_msgs/Quaternion orientation = [0,0,0,1]              
*
*   3. publishes to "wrist_joints" of type sensor_msgs::JointState with:
*           String[] name = ["w1", "w2", "w3"]
*           float64[] positon = [m1 [rad], m2 [rad], m3 [rad]]
*           float64[] velocity = []
*           float64[] effort = []
*/
int main(int argc, char **argv)
{

   //------------------------------ fill parameters with default values

    // base of wrist to motor 1 axis, in base frame
    link1(0) = 0; // dx [m]
    link1(1) = 0; // dy [m]
    link1(2) = 0; // dz [m]

    // motor 1 axis to motor 2 axis, in motor 1 frame
    link2(0) = 0.024; // dx [m]
    link2(1) = 0;     // dy [m]
    link2(2) = 0.0;   // dz [m]

    // motor 2 axis to tip of wrist, in motor 2 frame
    link3(0) = 0;     // dx [m]
    link3(1) = 0;     // dy [m]
    link3(2) = 0.0;   // dz [m]

    // tip of wrist to tip of end effector, in motor 3 frame
    end_effector(0) = 0; // dx [m]
    end_effector(1) = 0; // dy [m]
    end_effector(2) = 0; // dz [m]
    end_effector(3) = 1; // required for homogenous transformation

  //-------------------------------- initalize ros node
  
  // ros node stuff
  ros::init(argc, argv, "sphere_wrist_ik_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  std::string ns = ros::this_node::getNamespace();

  // ros parameters ------------------------------------------------------------------
  if(ros::param::get(ns + "/w1_x",link1(0))==false)
  {
		ROS_WARN("No parameter %s/w1_x, USING DEFAULT %f",ns.c_str(),link1(0));
  }
  if(ros::param::get(ns + "/w1_y",link1(1))==false)
  {
		ROS_WARN("No parameter %s/w1_y, USING DEFAULT %f",ns.c_str(),link1(1));
  }
  if(ros::param::get(ns + "/w1_z",link1(2))==false)
  {
		ROS_WARN("No parameter %s/w1_z, USING DEFAULT %f",ns.c_str(),link1(2));
  }
  if(ros::param::get(ns + "/w2_x",link2(0))==false)
  {
		ROS_WARN("No parameter %s/w2_x, USING DEFAULT %f",ns.c_str(),link2(0));
  }
  if(ros::param::get(ns + "/w2_y",link2(1))==false)
  {
		ROS_WARN("No parameter %s/w2_y, USING DEFAULT %f",ns.c_str(),link2(1));
  }
  if(ros::param::get(ns + "/w2_z",link2(2))==false)
  {
		ROS_WARN("No parameter %s/w2_z, USING DEFAULT %f",ns.c_str(),link2(2));
  }
  if(ros::param::get(ns + "/w3_x",link3(0))==false)
  {
		ROS_WARN("No parameter %s/w3_x, USING DEFAULT %f",ns.c_str(),link3(0));
  }
  if(ros::param::get(ns + "/w3_y",link3(1))==false)
  {
		ROS_WARN("No parameter %s/w3_y, USING DEFAULT %f",ns.c_str(),link3(1));
  }
  if(ros::param::get(ns + "/w3_z",link3(2))==false)
  {
		ROS_WARN("No parameter %s/w3_z, USING DEFAULT %f",ns.c_str(),link3(2));
  }

  // initalization -------------------------------------------------------------------
  
  // publisher and subscribers
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("wrist_joints", 10);
  ros::Publisher offset_pub = n.advertise<geometry_msgs::Pose>("arm_offset",10);
  ros::Subscriber pose_sub = n.subscribe("wrist_goal", 10, poseCallback);


   if (ns != "" && ns.front() == '/') {
     ns.erase(0, 1); // Remove the leading slash if present
   }

  // main loop--------------------------------------------
  while (ros::ok())
  {
     // run the inverse kinematics and calculate the arm offset if a goal pose is given
     if(got_goal){
         sphere_ik();
         // create and fill wrist joint state and arm offset messages
         sensor_msgs::JointState msg;
         msg.name = {ns + "/w1", ns + "/w2", ns + "/w3"};
         msg.position = {m1, m2, m3};
         joint_pub.publish(msg);


         calc_arm_offset();
         geometry_msgs::Pose offset;
         offset.position.x = arm_offset(0);
         offset.position.y = arm_offset(1);
         offset.position.z = arm_offset(2);
         offset.orientation.x = 0;
         offset.orientation.y = 0;
         offset.orientation.z = 0;
         offset.orientation.w = 1;

         // publish messages
         offset_pub.publish(offset);

         got_goal = false;
      } 

     
      // ros continue
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}


















