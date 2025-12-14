
#!/usr/bin/env python3
import rospy
from std_msgs.msg import  Float64
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
import numpy as np
import tf

import random

'''

what do

'''

#---------------------------------------------------- HELPER CLASSES ------------------------------------------------

# Initialize variables
ns = ""
common_frame = "stickbug/drivebase/base_link"
tf_listener = None

max_time_between_pollinations = 120 # seconds
conflict_distance = .1 # meters

# arm states 
ALL_FLOWERS_IN_COLLISION = 1 # red
GOING_TO_FLOWER = 2 # green
POLLINATING = 3 # yellow
BACKING_UP = 4  # blue
GOING_TO_TOLD_POSE = 5 # pink
NO_FLOWERS = 6 # blue-green
NO_FLOWERS_WITHIN_REACH = 7 # white

ee_points = [[0,0,0],
             [0,0,0],
             [0,0,0],
             [0,0,0],
             [0,0,0],
             [0,0,0]]

time_since_last_pollination = [0,0,0,0,0,0]

told_poses = [None, None, None, None, None, None]
arms_in_conflict = [False, False, False, False, False, False]

#---------------------------------------------------- CALBACK FUNCTINS ------------------------------------------------

# time since last pollination for each arm
def TSLP1_cb(data):
    global time_since_last_pollination
    time_since_last_pollination[0] = data.data

def TSLP2_cb(data):
    global time_since_last_pollination
    time_since_last_pollination[1] = data.data

def TSLP3_cb(data):
    global time_since_last_pollination
    time_since_last_pollination[2] = data.data

def TSLP4_cb(data):
    global time_since_last_pollination
    time_since_last_pollination[3] = data.data

def TSLP5_cb(data):
    global time_since_last_pollination
    time_since_last_pollination[4] = data.data

def TSLP6_cb(data):
    global time_since_last_pollination
    time_since_last_pollination[5] = data.data


#---------------------------------------------------- HELPER FUNCTIONS ------------------------------------------------

def conflict_detection_and_resolution():
    global arms_in_conflict, time_since_last_pollination, max_time_between_pollinations, told_poses, ee_points, conflict_distance
    
    # ================================ identify arms in conflict ===============================
    
    # determine arm ids in conflict
    for i in range(6):
        if time_since_last_pollination[i] > max_time_between_pollinations:
            arms_in_conflict[i] = True

    #============== Determine type of conflict and resolve it ==================================
            
    # manipulator-manipulator conflict-------------------------------------------- 
    #          based on if two arms are within a certain distance of each other
    for i in range(6):
        for j in range(i+1, 6):
            if arms_in_conflict[i] and arms_in_conflict[j] and distance(ee_points[i], ee_points[j]) < conflict_distance:
                # arms i and j are in manipulator-manipulator conflict
                # resolve conflict: send a random arm home
                arms_in_conflict[i] = False
                arms_in_conflict[j] = False
                if random.random() < 0.5:
                    # send arm i home
                    told_poses[i] = Pose()
                    told_poses[i].orientation.w = 0 # all zero orientation is home
                    told_poses[j] = None
                else:
                    # send arm j home
                    told_poses[j] = Pose()
                    told_poses[j].orientation.w = 0 # all zero orientation is home
                    told_poses[i] = None


    # manipulator environment conflict-------------------------------------------------------
    #     based on if an arm is in conflict but not near another arm
    for i in range(6):
        if arms_in_conflict[i]:
            # arm i is in manipulator-environment conflict
            # resolve conflict: send arm home
            arms_in_conflict[i] = False
            told_poses[i] = Pose()
            told_poses[i].orientation.w = 0 # all zero orientation is home
            told_poses[j] = None


    # drivebase manipulator conflict TODO


def calculate_distance(pose1, pose2):
    """Calculate the Euclidean distance between two poses."""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    return np.sqrt(dx*dx + dy*dy + dz*dz)

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def get_tf(target_frame, source_frame, listener):
    tf_listener = listener
    try:
        # Wait for the transform to become available
        tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(secs=0.05))
        
        # Look up the transform
        (trans, rot) = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(target_frame)
        print(source_frame)
        

def get_manipulator_joint_feedback(listener):
    global common_frame, ee_points

    # Assuming arm_ids is a list of all arm identifiers including the current arm's
    arm_ids = [1, 2, 3, 4, 5, 6]  # Example: You might want to dynamically generate this list

    for arm_id in arm_ids:
        # Construct namespace for the arm based on its ID
        arm_ns = "stickbug/arm{}/".format(arm_id)

        try:
            # Look up transform for each link in the arm
            end_pt, rot = get_tf(common_frame, arm_ns + 'end_link', listener)  # Assume rot needed only for end link
            ee_points[arm_id-1] = end_pt
           
        except Exception as e:
            rospy.logwarn("Arm{} is not online or transform not available.".format(arm_id))
           


#--------------------------------------------------- MAIN -------------------------------------------------------------

def referee_auto():
    global tf_listener, time_since_last_pollination, max_time_between_pollinations, told_poses, ee_points, arms_in_conflict

    # initialize node ----------------------------------------------------------
    rospy.init_node('referee_autonomy')
    rate = rospy.Rate(1)

    # get parameters -----------------------------------------------------------
    
    

    # publisher and subscribers ------------------------------------------------
    # Subscribers
    rospy.Subscriber("stickbug/arm1/time_since_last_pollination", Float64, TSLP1_cb)
    rospy.Subscriber("stickbug/arm2/time_since_last_pollination", Float64, TSLP2_cb)
    rospy.Subscriber("stickbug/arm3/time_since_last_pollination", Float64, TSLP3_cb)
    rospy.Subscriber("stickbug/arm4/time_since_last_pollination", Float64, TSLP4_cb)
    rospy.Subscriber("stickbug/arm5/time_since_last_pollination", Float64, TSLP5_cb)
    rospy.Subscriber("stickbug/arm6/time_since_last_pollination", Float64, TSLP6_cb)

    # Publishers
    told_arm1_pose_pub = rospy.Publisher("stickbug/arm1/told_pose", Pose, queue_size=10)
    told_arm2_pose_pub = rospy.Publisher("stickbug/arm2/told_pose", Pose, queue_size=10)
    told_arm3_pose_pub = rospy.Publisher("stickbug/arm3/told_pose", Pose, queue_size=10)
    told_arm4_pose_pub = rospy.Publisher("stickbug/arm4/told_pose", Pose, queue_size=10)
    told_arm5_pose_pub = rospy.Publisher("stickbug/arm5/told_pose", Pose, queue_size=10)
    told_arm6_pose_pub = rospy.Publisher("stickbug/arm6/told_pose", Pose, queue_size=10)
    

    
    # Initialize ------------------------------------------------------------------------
    tf_listener = tf.TransformListener()
    get_manipulator_joint_feedback(listener=tf_listener)
   

    
    # main loop -----------------------------------------------------------------
    while not rospy.is_shutdown():
        
        # get manipulator joint feedback
        get_manipulator_joint_feedback(listener=tf_listener)
        # detect conflicts and resolve them
        conflict_detection_and_resolution()

        # publish messages--------------------------------------------------------

        # send out resolutions if any 
        for i in range(6):
            if told_poses[i] is not None:
                if i == 0:
                    told_arm1_pose_pub.publish(told_poses[i])
                elif i == 1:
                    told_arm2_pose_pub.publish(told_poses[i])
                elif i == 2:
                    told_arm3_pose_pub.publish(told_poses[i])
                elif i == 3:
                    told_arm4_pose_pub.publish(told_poses[i])
                elif i == 4:
                    told_arm5_pose_pub.publish(told_poses[i])
                elif i == 5:
                    told_arm6_pose_pub.publish(told_poses[i])
                
                told_poses[i] = None
        
        
        # continue to loop -------------------------------------------------------
        rate.sleep()
    

if __name__ == '__main__':
    try:
      referee_auto()
    except rospy.ROSInterruptException:
       pass














