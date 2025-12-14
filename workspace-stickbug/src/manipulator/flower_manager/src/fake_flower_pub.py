import rospy
from std_msgs.msg import Int64, Float64, Bool
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import tf.transformations as tf_trans  # Import for quaternion conversion

import time


def nullify_roll(quaternion):
    """
    Removes the roll component from a quaternion while keeping yaw and pitch.
    
    Args:
        quaternion (list or tuple): Quaternion [x, y, z, w]

    Returns:
        list: New quaternion with roll set to zero
    """
    # Convert quaternion to Euler angles
    roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion, axes='sxyz')

    # Set roll to zero
    roll = 0.0

    # Convert back to quaternion
    new_quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
    
    return new_quaternion


common_frame = "base"

def fake_flower_pub():

    global arm_state, num_flowers_pollinated, time_since_last_pollination, common_frame

    # initialize node ----------------------------------------------------------
    rospy.init_node('fake_flower_pub')
    rate = rospy.Rate(10)

    ns= rospy.get_namespace()  # Get the current namespace

    # Construct the parameter name dynamically
    param_name = ns + "common_frame"  # Assuming 'u2d2' is the parameter you want to access
    # Get a parameter, including its namespace, with a default value if the parameter is not found
    param_val = rospy.get_param(param_name, common_frame)

  

    if param_val  == common_frame:
        rospy.logwarn(f"The default value for 'common_frame' is being used: {common_frame}")
    common_frame = param_val
    print('Parameter value:',  common_frame )


    # publisher and subscribers ------------------------------------------------
    
    #                       HARD CODE OVERRIDE VALUES FOR TESTING
    # Publishers
    flower_pub = rospy.Publisher("/stickbug/arm6/raw_flowers", PoseArray, queue_size=10)
    common_frame = "stickbug/drivebase/r_lift"
    
    home_pose = Pose()
    home_pose.position.x = 0.3
    home_pose.position.y = -0.25
    home_pose.position.z = 1.1
    home_pose.orientation.x = 0.0
    home_pose.orientation.y = 0.0
    home_pose.orientation.z = .3824995  # (+)45 deg rotation yaw
    home_pose.orientation.w = 0.9239557

    print(f"real topic: /stickbug/arm6/raw_flowers")
    print(f"real common frame : stickbug/drivebase/r_lift")
    

    # Initialize ------------------------------------------------------------------------
    flowers = PoseArray()
    flowers.header.frame_id = common_frame
    for i in range(1):
        flower = Pose()
        flower.position = home_pose.position
        flower.orientation = home_pose.orientation
        
        # y check
        # flower.position.x = 0.35
        # flower.position.y = 0.0

        # yaw check
        # flower.position.x = .35
        # flower.position.y = -.35
        # flower.orientation.z = 0.0
        # flower.orientation.w = 1.0

        # xy yaw check
        flower.position.x = .45
        flower.position.y = 0.0
        # flower.orientation.y = -.3824995
        # flower.orientation.z = 0.0
        # flower.orientation.w = 0.9239557

        # working one
        # flower_quat = np.array([0, -.3824995, 0, 0.9239557])

        # print(flower_quat)
        # flower_euler = tf_trans.euler_from_quaternion(flower_quat)
        # print(flower_euler)

        # flower_quat = tf_trans.quaternion_from_euler(flower_euler[0], flower_euler[1], flower_euler[2])
        # print(flower_quat)
        # print('---')


        # orientation copied from actual flower
        flower_quat = np.array([-0.974033225005351, -0.17050814686579113, 0.1189765161468388, 0.08961493761438673])

        flower_quat = nullify_roll(flower_quat)

        # new test pose
        # flower_quat = np.array([ 0.1464466, 0.3535534, 0.3535534, 0.8535534 ])

        # new test pose
        # flower_quat = np.array([ 0.863069, -0.3213938, 0.3830222, -0.0714973 ])


        print(flower_quat)
        flower_euler = np.array(tf_trans.euler_from_quaternion(flower_quat))

        # flower_euler[0] =  flower_euler[0] + 3.1415
        
        print(flower_euler)

        flower_quat = tf_trans.quaternion_from_euler(flower_euler[0], flower_euler[1], flower_euler[2])
        print(flower_quat)
        print('---')

        # flower_quat = remove_quaternion_loops(flower_quat)

        flower.orientation.x = flower_quat[0]
        flower.orientation.y = flower_quat[1]
        flower.orientation.z = flower_quat[2]
        flower.orientation.w = flower_quat[3]


    
        # flower.position.x = 0.4 #0.3 + i*0.015
        # flower.position.y = 0.1 - i*0.02 
        # flower.position.z = 1.1
        # flower.orientation.x = 0.0
        # flower.orientation.y = 0.0
        # flower.orientation.z = 0.0
        # flower.orientation.w = 1.0
        flowers.poses.append(flower)

    # main loop -----------------------------------------------------------------
    while not rospy.is_shutdown():
        
        # publish messages 
        flower_pub.publish(flowers)
        
        
        # continue to loop -------------------------------------------------------
        rate.sleep()
    

if __name__ == '__main__':
    try:
      fake_flower_pub()
    except rospy.ROSInterruptException:
       pass









