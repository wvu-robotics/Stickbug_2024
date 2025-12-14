
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32MultiArray
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
import copy
import re
import sys
import tf.transformations as tf_trans


#-------------------------------------------------- Initialize variables ---------------------------------------------

tf_listener = None
common_frame = 'base'
my_id = 0
ns = ''

move_group = None
move_group_names = [ 'left_arm_1',    # arm 1 
                    'right_arm_1',    # arm 2
                     'left_am_2',    # arm 3
                    'right_arm_2',    # arm 4
                     'left_arm_3',    # arm 5
                    'right_arm_3']    # arm 6

Ki = 0.001 #7e-6
Kp = .55 #0.005/4

got_goal = False
got_joints = False

goal_pose = Pose()
current_pose = Pose()

actual_joints = JointState()
goal_joints = JointState()
zero_velocity_joints = JointState()

num_joint = None

reached_goal_threshold = 0.01

#---------------------------------------------------- CALBACK FUNCTIONS ------------------------------------------------
def goal_callback(msg):
    global goal_pose, got_goal
    goal_pose = msg
    got_goal = True

def joint_state_callback(msg):
    global actual_joints,move_group, current_pose,zero_velocity_joints, num_joint, tf_listener, common_frame, got_joints
    global ns
    actual_joints = msg

    num_joint = len(actual_joints.position)

    zero_velocity_joints = copy.deepcopy(actual_joints)
    zero_velocity_joints.velocity = [0.0]*num_joint

    end_pt, rot = get_tf(common_frame, ns + 'end_link')

    current_pose.position.x = end_pt[0]
    current_pose.position.y = end_pt[1]
    current_pose.position.z = end_pt[2]
    current_pose.orientation.x = rot[0]
    current_pose.orientation.y = rot[1]
    current_pose.orientation.z = rot[2]
    current_pose.orientation.w = rot[3]
    
    got_joints = True


#--------------------------------------------------- HELPER FUNCTIONS ------------------------------------------------
def servo():
    global goal_pose, current_pose, move_group, actual_joints, Ki, Kp, goal_joints, num_joint

    # calculate the error
    error_pos = np.array([goal_pose.position.x - current_pose.position.x, 
                          goal_pose.position.y - current_pose.position.y, 
                          goal_pose.position.z - current_pose.position.z])
    

    # Convert goal and current orientations from quaternion to Euler angles (roll, pitch, yaw)
    goal_euler = quaternion_to_euler(goal_pose.orientation)
    current_euler = quaternion_to_euler(current_pose.orientation)
    
    # Calculate orientation error (considering yaw and pitch)
    error_orient = np.array([ 0.0,                            # no roll error                                                      
                             goal_euler[1] - current_euler[1],  # pitch error
                             goal_euler[2] - current_euler[2]])  # yaw error

    # Combine position and orientation error
    error = np.concatenate((error_pos, error_orient))

    # calculate the jacobian
    Jacobian = move_group.get_jacobian_matrix(list(actual_joints.position))
    J_pos = Jacobian #[0:3,:]
    # calculate the joint velocities
    joint_velocities = np.dot(np.linalg.pinv(J_pos), (Kp * error))
    # calculate the joint positions
    goal_joints = copy.deepcopy(actual_joints)
    new_pos = [actual_joints.position[i] + .1*joint_velocities[i] for i in range(len(joint_velocities))]
    goal_joints.position = [new_pos[0], new_pos[1], new_pos[2], new_pos[3], new_pos[4], 0.0, 0.0, 0.0]

    goal_joints.position[0] = goal_pose.position.z-.05 # set stepper goal

    goal_joints.velocity = [joint_velocities[0], joint_velocities[1], joint_velocities[2], joint_velocities[3], joint_velocities[4], 0.0,0.0,0.0]
    goal_joints.header.stamp = rospy.Time.now()

def calculate_distance(pose1, pose2):
    """Calculate the Euclidean distance between two poses."""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    return np.sqrt(dx*dx + dy*dy + dz*dz)

def get_tf(target_frame, source_frame):
    global tf_listener
    try:
        # Wait for the transform to become available
        tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(secs=0.05))
        
        # Look up the transform
        (trans, rot) = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        pass

def quaternion_to_euler(quaternion):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw)
    """
    euler = tf_trans.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return euler  # returns a tuple (roll, pitch, yaw)


#--------------------------------------------------- MAIN -------------------------------------------------------------

def pose_servo():
    global tf_listener, my_id, common_frame, move_group, goal_pose, actual_joints, got_goal, reached_goal_threshold
    global ns

    # initialize node ----------------------------------------------------------
    rospy.init_node('pose_servo')
    rate = rospy.Rate(1)

    # Check if at least one argument is provided
    if len(sys.argv) > 1:
        try:
            # Convert the first argument to an integer
            my_id = int(sys.argv[1])
        except ValueError:
            rospy.logerr("First argument must be an integer")
            sys.exit(1)
    else:
        rospy.logerr("No ID argument provided")
        my_id = 1

    namespace = '/stickbug/arm' + str(my_id) + '/'

    # Construct the parameter name dynamically
    param_name = namespace + "common_frame" 
    param_val = rospy.get_param(param_name, common_frame)
    if param_val  == common_frame:
        rospy.logwarn(f"The default value for 'common_frame' is being used: {common_frame}")
    common_frame = param_val
    print('Parameter value:',  common_frame )

    if namespace[0] == '/':
        ns = namespace[1:]
    else:
        ns = namespace

    # publisher and subscribers ------------------------------------------------
    # Subscribers
    rospy.Subscriber(namespace + 'manipulator_goal',Pose, goal_callback)
    rospy.Subscriber(namespace +'manipulator_actual_joints', JointState, joint_state_callback)
    
    # Publishers
    joint_pub = rospy.Publisher(namespace +'manipulator_goal_joints', JointState, queue_size=10)
  
    # Initialize ------------------------------------------------------------------------
    tf_listener = tf.TransformListener()
    move_group = MoveGroupCommander(move_group_names[my_id-1],wait_for_servers=30.0)
    

    # main loop -----------------------------------------------------------------
    while not rospy.is_shutdown():
        
        if got_goal and got_joints:
            if calculate_distance(goal_pose, current_pose) < reached_goal_threshold:
                got_goal = False
                joint_pub.publish(zero_velocity_joints )
                print('Reached goal')
            else:
                servo()
                print('goal_joints velocity:', goal_joints.velocity)
                # publish the joint positions
                joint_pub.publish(goal_joints)
        
           
         
        # continue to loop -------------------------------------------------------
        rate.sleep()
    

if __name__ == '__main__':
    try:
      pose_servo()
    except rospy.ROSInterruptException:
       pass


