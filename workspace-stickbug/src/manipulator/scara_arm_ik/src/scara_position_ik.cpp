#include <math.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <string>

# define M_PI           3.14159265358979323846

using namespace std;
using namespace Eigen;

/*###################################################################### Overview #####################################################################################
 *                             
 *  Description: This node is used for POSITION INVERSE KINEMATICS of a standard SCARA arm. Where the first joint is a prismatic lift, the second a rotary joint whose
 *               axis is parallel to the lift, and a third joint that is rotary who is also parallel to the lift.  
 *  
 *  NOTE: This node does NOT handle any rotations of the end effector!!!! that should be done by a subsequent WRIST node to decouple the Inverse kinematics.
 * 
 *=====================================================================================================================================================================
 *                       TOP VIEW
 *
 *        ^ Y                          __End effector             |                            
 *        |                           /                           |           /| <-- end effector     1. convert from cartestion to polar R and theta                 
 *        |                         V ---|                        |          / |                         - R is measured from shoulder motor axis to end effector tip
 *        o-----> X                 |    |                        |         /  |                         - theta is measured from (+)X axis to R
 *       Z         origin           |    |  forearm_length        |        /   |                      2. alpha is the shoulder motor angle
 *                    |             |    |                        |    R  /    |                          - measured from (+)X to bicep_link
 *                    V             |    |                        |      /     |                          - positive rotation for (-)Z rotation
 *   shoulder motor-> O=============O ---|                        |     /      |                      3. beta is the elbow motor angle
 *                    |             | \                           |    /       |                         - measured from bicep_link to elbow_link
 *                    |-------------|  \__elbow motor             |   / theta  |                         - positive rotation for (-)Z rotation
 *                     bicep_length                               |  /)_______(| <-beta               4. use Law of Sines and Law of Cosines to calculate angles
 *                                                                |  ^
 * ===============================================================|  Shoulder motor
 *                   BACK VIEW                                    |  
 *        ^ Z                                                     |   ex) x = 1,  y =   0, z = 2    --> [2,  0  ,  3.14] ==> [2, 1.57, -1.57]
 *        |                                                       |   ex) x = 0,  y =  -1, z = 3    --> [3,  1.57, 3.14] ==> [3, 0, -1.57]
 *    <---x X          |                                          |   ex) x = .5, y = -.5, z = 4    --> [4,  1.57, 1.57] ==> [4, 0, 0]
 *    Y                |                                          |
 *                 ---[o]=====o----<                              |
 *     lift_pose    |  |                                          |
 *                  |  |                                          |
 *                 --- |  <--- origin                             |
 *                                                                |
 *                                                                |
*/
//################################################################## Global Variables #################################################################################

double bicep_length = .5;    // length of first link from shaft of shoulder motor to elbow motor in [m]
double forearm_length = .5;  // length of second link from shaft of elbow motor to end effector tip in [m]

double lift_pose = 0;        // lift motor joint goal location in [m]
double shoulder_angle = 0;   // shoulder motor joint goal angle in [rad] 
double elbow_angle = 0;      // elbow motor joint goal angle in [rad]

tf2::Quaternion wrist_offset;  // the direction the tip of the arm points in about the Z axis [rad]
VectorXd pose(3);
MatrixXd Rz(3,3);

bool is_right_arm = true;     // if the arm kinematics are a right or left arm, effects the yaw-offset 
bool got_goal = false;

double collar_bone_x = .102;
double collar_bone_y = -.0444;
double collar_bone_z = 0.05;
double collar_bone_yaw = M_PI/6; // 30 degrees

//################################################################# Position callback ##################################################################################

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

  

  pose(0) = msg->position.x - collar_bone_x;  //not typos the orignal coordinate frame for the equations was X horizontal and Y forward
  pose(1) = msg->position.y - collar_bone_y;
  pose(2) = msg->position.z - collar_bone_z;
  

  // rotate from base frame to shoulder frame
  pose = Rz*pose;

  //ROS_INFO("x = %f, y = %f, z = %f", pose(0), pose(1), pose(2));

  double x = pose(0);
  double y = pose(1);
  double z = pose(2);

  if(!is_right_arm) y = y; // mirror the problem for left hand
  
  double R = sqrt(pow(x, 2.0) + pow(y, 2.0)); // length from shoulder to goal
  double theta = atan2(y , x);  // angle from (+)X to R

  //ROS_INFO("theta= %f, y=%f, x=%f", theta, y, x);
  

  // calculate target angle for elbow
  double beta = pow(R, 2.0) - pow(bicep_length, 2.0) - pow(forearm_length, 2.0);
         beta /= -2 * bicep_length * forearm_length;
         beta = acos(beta);

  // calculate target angle for shoulder
  double gamma = asin(sin(beta)*forearm_length/R);
  double alpha = theta - gamma ;

  // calculate wrist offset 
  double yaw = alpha + (M_PI - beta) - collar_bone_yaw -11.74*M_PI/180;

  if(!is_right_arm){
    alpha = theta + gamma;
    yaw = alpha - (M_PI - beta) - collar_bone_yaw +11.74*M_PI/180;

  }

  //ROS_INFO("The yaw offset = %f alpha = %f beta = %f, theta= %f", yaw, alpha, beta, theta);
  
  if (!std::isnan(alpha) && !std::isnan(beta)) {
    
    shoulder_angle = alpha - 11.74*M_PI/180;
    elbow_angle = M_PI / 2 - beta;

    if (!is_right_arm) {
      shoulder_angle = alpha + 11.74*M_PI/180;
      elbow_angle = beta - M_PI/2;   
    } 
    
    lift_pose = z;
    wrist_offset.setRPY(0,0,-yaw);
    wrist_offset.normalize();
    
  }
  got_goal = true;
  

}


/*################################################################## MAIN #################################################################################
*
*   subscribes to "arm_goal" of type geometry_msgs::Pose
*   extracts XYZ position from msg and calculates motor joint angles
*
*   publishes to "wrist_offset" of type geometry_msgs::Pose
*           Geometry_msgs/Point position = [0, 0, 0]      
*           Geometry_msgs/Quaternion orientation = [yaw_offset as quaternion]   
*     
*   publishes to "arm_joints" of type sensor_msgs::JointState with:
*       String[] name = ["lift", "shoulder", "elbow"]
*       float64[] positon = [lift position[m], shoulder[rad], elbow[rad]]
*       float64[] velocity = []
*       float64[] effort = []
*/
int main(int argc, char **argv)
{

  // initialize wrist offset
  wrist_offset.setRPY(0,0,0);
  wrist_offset.normalize();
  
  // ros node stuff
  ros::init(argc, argv, "scara_pos_ik_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  std::string ns = ros::this_node::getNamespace();

  // ros parameters --------------------------------------------------------------------------------
  if(ros::param::get(ns + "/is_right_arm",is_right_arm)==false)
  {
		ROS_WARN("No parameter %s/is_right_arm specified, using default right arm", ns.c_str());
  }
  if(ros::param::get(ns + "/bicep_length",bicep_length)==false)
  {
		ROS_WARN("No parameter %s/bicep_length specified, using default %f", ns.c_str(), bicep_length);
  }
  if(ros::param::get(ns + "/forearm_length",forearm_length)==false)
  {
		ROS_WARN("No parameter %s/forearm_length specified, using default %f", ns.c_str(), forearm_length);
  }
  if(ros::param::get(ns + "/collar_bone_x",collar_bone_x)==false)
  {
		ROS_WARN("No parameter %s/collar_bone_x specified, using default %f", ns.c_str(), collar_bone_x);
  }
  if(ros::param::get(ns + "/collar_bone_y",collar_bone_y)==false)
  {
		ROS_WARN("No parameter %s/collar_bone_y specified, using default %f", ns.c_str(), collar_bone_y);
  }
  if(ros::param::get(ns + "/collar_bone_z",collar_bone_z)==false)
  {
		ROS_WARN("No parameter %s/collar_bone_z specified, using default %f", ns.c_str(), collar_bone_z);
  }
  if(ros::param::get(ns + "/collar_bone_yaw",collar_bone_yaw)==false)
  {
		ROS_WARN("No parameter %s/collar_bone_yaw specified, using default %f", ns.c_str(), collar_bone_yaw);
  }
  // instantiation ---------------------------------------------------------------------------------
  
  // publisher and subscribers
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("arm_joints", 10);
  ros::Publisher wrist_pub = n.advertise<geometry_msgs::Pose>("wrist_offset", 10);
  ros::Subscriber pose_sub = n.subscribe("arm_goal", 10, poseCallback);

  if(!is_right_arm){
    collar_bone_yaw = -collar_bone_yaw;
    collar_bone_y = -collar_bone_y;
  }

  // rotatation from base frame to shoulder frame about Z axis
  Rz(0,0) = cos(collar_bone_yaw);    Rz(0,1) = -sin(collar_bone_yaw);   Rz(0,2) = 0;    
	Rz(1,0) = sin(collar_bone_yaw);    Rz(1,1) = cos(collar_bone_yaw);    Rz(1,2) = 0;     
	Rz(2,0) = 0;                       Rz(2,1) = 0;                       Rz(2,2) = 1;    
	

  if (ns != "" && ns.front() == '/') {
    ns.erase(0, 1); // Remove the leading slash if present
  }

  // main loop
  while (ros::ok())
  {
    
    if(got_goal){
      // create and fill message
      sensor_msgs::JointState msg;
      msg.name = {ns + "/slider", ns + "/shoulder", ns + "/elbow"};
      msg.position = {lift_pose, shoulder_angle, elbow_angle};
      joint_pub.publish(msg);

      geometry_msgs::Pose offset;
      offset.position.x = 0;
      offset.position.y = 0;
      offset.position.z = 0;
      offset.orientation.x = wrist_offset.x();
      offset.orientation.y = wrist_offset.y();
      offset.orientation.z = wrist_offset.z();
      offset.orientation.w = wrist_offset.w();
  
    // publish message
    wrist_pub.publish(offset);

      got_goal = false;
    }

    
    

    // ros continue
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
























