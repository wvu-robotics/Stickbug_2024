import rospy
from std_msgs.msg import Int64, Float64, Bool
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import numpy as np
import tf
import time
import re
'''

what do

'''

# -------------------------------------------------- helper classes ---------------------------------------------
class Flower:
    def __init__(self, pose, frame_name,time_out=1,timetopoll = 60):
        # flower members 
        self.pose_stamped = PoseStamped()
        self.pollinated = False
        self.expiration_time = rospy.Time.now() + rospy.Duration(10)
        self.time_to_pollinate =  600 #timetopoll

        # fill out pose stamped
        self.pose_stamped.pose = pose
        self.pose_stamped.header.stamp = rospy.Time.now()
        self.pose_stamped.header.frame_id = frame_name
    
    def add_time(self, dt):
        # Add additional time to the expiration time
        self.expiration_time += rospy.Duration(dt)
    
    def expired(self):
        # Check if the current time is greater than the expiration time
        return rospy.Time.now() > self.expiration_time
    
    def can_pollinate(self):
    	# Check if the flower is not pollinated and has enough time left to be pollinated
        if not self.pollinated:
            # Calculate the remaining time before the flower expires
            remaining_time_before_expiration = (self.expiration_time - rospy.Time.now()).to_sec()
            # Check if there's enough time to pollinate
            return remaining_time_before_expiration > self.time_to_pollinate
        else:
            return False

# -------------------------------------------------- Initialize variables ---------------------------------------------

tf_listener = None

Flowers = []
flower_distance_threshold = 0.05
common_frame = 'base'
additional_time = 10 #1 # seconds
my_id = 0


#---------------------------------------------------- CALBACK FUNCTINS ------------------------------------------------
def raw_flowers_callback(msg):
    global Flowers, tf_listener, flower_distance_threshold, common_frame, additional_time
    
    # Iterate through each pose in the PoseArray message
    for pose in msg.poses:
        # Create a PoseStamped message from the Pose message for transformation
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = msg.header.frame_id
        pose_stamped.header.stamp = rospy.Time(0)  # Use the time at which the pose was observed

        # Transform the pose to the common frame, if necessary
        try:
            if msg.header.frame_id != common_frame:
                # Wait for the transformation to be available and transform the pose
                transformed_pose_stamped = tf_listener.transformPose(common_frame, pose_stamped)
            else:
                transformed_pose_stamped = pose_stamped
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Error transforming pose to common frame: %s' % str(e))
            continue
        
        # Check if the pose is close to any existing flower
        flower_found = False
        for flower in Flowers:
            if calculate_distance(flower.pose_stamped.pose, transformed_pose_stamped.pose) <= flower_distance_threshold:
                # Update the existing flower's pose and timestamp
                # flower.pose_stamped = transformed_pose_stamped # <_ TODO ADD FILTER FOR THE POSITON and ORIENTATION
                flower.pose_stamped.header = transformed_pose_stamped.header

                flower.pose_stamped.pose.position.x = 0.9*flower.pose_stamped.pose.position.x + 0.1*transformed_pose_stamped.pose.position.x
                flower.pose_stamped.pose.position.y = 0.9*flower.pose_stamped.pose.position.y + 0.1*transformed_pose_stamped.pose.position.y
                flower.pose_stamped.pose.position.z = 0.9*flower.pose_stamped.pose.position.z + 0.1*transformed_pose_stamped.pose.position.z
                flower.pose_stamped.pose.orientation.x = 0.9*flower.pose_stamped.pose.orientation.x + 0.1*transformed_pose_stamped.pose.orientation.x
                flower.pose_stamped.pose.orientation.y = 0.9*flower.pose_stamped.pose.orientation.y + 0.1*transformed_pose_stamped.pose.orientation.y
                flower.pose_stamped.pose.orientation.z = 0.9*flower.pose_stamped.pose.orientation.z + 0.1*transformed_pose_stamped.pose.orientation.z
                flower.pose_stamped.pose.orientation.w = 0.9*flower.pose_stamped.pose.orientation.w + 0.1*transformed_pose_stamped.pose.orientation.w

                flower_found = True
                flower.add_time(additional_time)
                break
        
        if not flower_found:
            # Add a new flower to the list
            new_flower = Flower(transformed_pose_stamped.pose, common_frame,additional_time)
            Flowers.append(new_flower)

def pollinated_callback(msg):
    global Flowers, tf_listener, flower_distance_threshold, common_frame, my_id

    arm_id = int(msg.header.frame_id[-1])
    actual_frame = msg.header.frame_id[:-1]

    # Iterate through each pose in the PoseArray message
    for pose in msg.poses:
        # Create a PoseStamped message from the Pose message for transformation
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = actual_frame
        pose_stamped.header.stamp = rospy.Time(0)  # Use the time at which the pose was observed
        
        # Transform the pose to the common frame, if necessary
        try:
            if msg.header.frame_id != common_frame:
                # Wait for the transformation to be available and transform the pose
                transformed_pose_stamped = tf_listener.transformPose(common_frame, pose_stamped)
            else:
                transformed_pose_stamped = pose_stamped
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Error transforming pose to common frame: %s' % str(e))
            continue

        # Find the closest flower to the pose
        closest_flower = None
        min_distance = float('inf')
        for flower in Flowers:
            distance = calculate_distance(flower.pose_stamped.pose, transformed_pose_stamped.pose)
            if distance < min_distance:
                min_distance = distance
                closest_flower = flower
        
        # If the closest flower is within the threshold, set it as pollinated
        if closest_flower and min_distance <= flower_distance_threshold:
            closest_flower.pollinated = True
            closest_flower.pose_stamped.pose = transformed_pose_stamped.pose
            closest_flower.pose_stamped.header.stamp = rospy.Time.now()
        # if it was not found but it is not the same arm, create a new flower and set it to pollinated
        elif arm_id != my_id:
            new_flower = Flower(transformed_pose_stamped.pose, common_frame)
            new_flower.pollinated = True
            Flowers.append(new_flower)
        # otherwise, we have a miscalibration that we cant find the flower we just pollinated
        else:
            rospy.logwarn('No flower found within the threshold distance')

#---------------------------------------------------- HELPER FUNCTIONS ------------------------------------------------
def calculate_distance(pose1, pose2):
    """Calculate the Euclidean distance between two poses."""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    return np.sqrt(dx*dx + dy*dy + dz*dz)

def get_poll_and_unpoll_flowers():
    global Flowers
    pollinated_flowers = []
    un_pollinated_flowers = []

    for flower in Flowers:
            if flower.pollinated:
                pollinated_flowers.append(flower)
            elif flower.can_pollinate():
                un_pollinated_flowers.append(flower)

    return pollinated_flowers, un_pollinated_flowers

def flowers_to_pose_array(flowers):
    # Initialize the PoseArray message
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()  # Timestamp the message with the current time
    if flowers:
        pose_array.header.frame_id = flowers[0].pose_stamped.header.frame_id  # Use the frame_id of the first flower

    # Iterate through the flowers and add their poses to the PoseArray
    for flower in flowers:
        pose_array.poses.append(flower.pose_stamped.pose)

    return pose_array

def prune_flowers(Flowers):
    # Use a list comprehension to filter out expired flowers
    pruned_flowers = [flower for flower in Flowers if not flower.expired()]
    return pruned_flowers

#--------------------------------------------------- MAIN -------------------------------------------------------------

def flower_manager():
    global tf_listener, my_id, common_frame, Flowers


    # initialize node ----------------------------------------------------------
    rospy.init_node('flower_manager')
    rate = rospy.Rate(1)

    # Extract arm number from namespace
    namespace = rospy.get_namespace()  # Get the current namespace
    my_id_match = re.search(r'arm(\d+)', namespace)
    if my_id_match:
        my_id = int(my_id_match.group(1))
    else:
        rospy.logwarn("Could not find arm number in namespace: {}".format(namespace))
        my_id = 0  # or set a default value

    # Construct the parameter name dynamically
    param_name = namespace + "common_frame"  # Assuming 'u2d2' is the parameter you want to access
    # Get a parameter, including its namespace, with a default value if the parameter is not found
    param_val = rospy.get_param(param_name, common_frame)



    if param_val  == common_frame:
        rospy.logwarn(f"The default value for 'common_frame' is being used: {common_frame}")
    common_frame = param_val
    print('Parameter value:',  common_frame )

    # publisher and subscribers ------------------------------------------------
    # Subscribers
    rospy.Subscriber("raw_flowers", PoseArray,raw_flowers_callback)
    rospy.Subscriber("/stickbug/newly_pollinated_flowers", PoseArray,pollinated_callback)

    # Publishers
    unpoll_flower_pub = rospy.Publisher("unpollenated_flowers", PoseArray, queue_size=10)
    poll_flower_pub = rospy.Publisher("pollenated_flowers", PoseArray, queue_size=10)

    # Initialize ------------------------------------------------------------------------
    tf_listener = tf.TransformListener()
    unpoll_flowers = PoseArray()
    poll_flowers = PoseArray()
    
    # main loop -----------------------------------------------------------------
    while not rospy.is_shutdown():
        
        # prune expired flowers
        Flowers = prune_flowers(Flowers)

        # get poll and unpoll flowers
        pollinated_flowers, un_pollinated_flowers = get_poll_and_unpoll_flowers()
        unpoll_flowers = flowers_to_pose_array(un_pollinated_flowers)
        poll_flowers = flowers_to_pose_array(pollinated_flowers)

        # publish messages
        unpoll_flower_pub.publish(unpoll_flowers)
        poll_flower_pub.publish(poll_flowers)
         
        # continue to loop -------------------------------------------------------
        rate.sleep()
    

if __name__ == '__main__':
    try:
      flower_manager()
    except rospy.ROSInterruptException:
       pass








