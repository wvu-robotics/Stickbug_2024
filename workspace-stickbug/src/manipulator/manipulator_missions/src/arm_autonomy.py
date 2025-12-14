
import rospy
from std_msgs.msg import Int64, Float64, Bool
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import time
import copy
import tf.transformations as tft
import re

'''

what do

'''

#---------------------------------------------------- HELPER CLASSES ------------------------------------------------

class TriangularPrism:
    def __init__(self, p1, p2, p3, z_buffer, buffer=0, color="b"):
        self.buffer = buffer
        self.color = color
        self.z_min = min(p1[2], p2[2], p3[2]) - z_buffer
        self.z_max = max(p1[2], p2[2], p3[2]) + z_buffer

        if buffer > 0:
            self.p1, self.p2, self.p3 = self.create_buffered_triangle(np.array(p1), np.array(p2), np.array(p3), buffer)
        else:
            self.p1, self.p2, self.p3 = np.array(p1), np.array(p2), np.array(p3)


    def plot(self,fig, ax, plot):
        # Vertices of the triangular prism
        vertices = [
            [self.p1[0], self.p1[1], self.z_min],
            [self.p2[0], self.p2[1], self.z_min],
            [self.p3[0], self.p3[1], self.z_min],
            [self.p1[0], self.p1[1], self.z_max],
            [self.p2[0], self.p2[1], self.z_max],
            [self.p3[0], self.p3[1], self.z_max]
        ]

        # Define the sides of the prism
        sides = [
            [vertices[0], vertices[1], vertices[4], vertices[3]], # Side 1
            [vertices[1], vertices[2], vertices[5], vertices[4]], # Side 2
            [vertices[2], vertices[0], vertices[3], vertices[5]], # Side 3
            [vertices[3], vertices[4], vertices[5]], # Top triangle
            [vertices[0], vertices[1], vertices[2]]  # Bottom triangle
        ]

        # Create a Poly3DCollection for the sides
        prism = Poly3DCollection(sides, facecolors=self.color, linewidths=1, alpha=.25)
        ax.add_collection3d(prism)

        # Set plot limits
        ax.set_xlim([min(self.p1[0], self.p2[0], self.p3[0]), max(self.p1[0], self.p2[0], self.p3[0])])
        ax.set_ylim([min(self.p1[1], self.p2[1], self.p3[1]), max(self.p1[1], self.p2[1], self.p3[1])])
        ax.set_zlim([self.z_min, self.z_max])

        # Plot the prism
        if plot:
            plt.show()
        

    def contains(self, point):
        point = np.array(point)
        in_triangle = self._point_in_triangle(point[:2], self.p1[:2], self.p2[:2], self.p3[:2])
        in_z_range = self.z_min <= point[2] <= self.z_max
        return in_triangle and in_z_range

    def _point_in_triangle(self, p, p0, p1, p2):
        # Barycentric coordinate method
        def sign(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

        b1 = sign(p, p0, p1) < 0.0
        b2 = sign(p, p1, p2) < 0.0
        b3 = sign(p, p2, p0) < 0.0

        return ((b1 == b2) and (b2 == b3))
    
    def unit_vector(self,vector):
        """Returns the unit vector of the vector."""
        return vector / np.linalg.norm(vector)

    def perpendicular_vector(self,v):
        """Finds an arbitrary perpendicular vector to v."""
        if v[0] == 0 and v[1] == 0:
            if v[2] == 0:
                # v is zero vector
                raise ValueError('zero vector')
            # v is a vector in the z-direction
            return np.cross(v, [0, 1, 0])
        return np.cross(v, [0, 0, 1])

    def create_buffered_triangle(self, p1, p2, p3, buffer_distance):
        p1, p2, p3 = np.array(p1), np.array(p2), np.array(p3)

        # Calculate the cross product of vectors (p2 - p1) and (p3 - p1)
        cross_product = np.cross(p2 - p1, p3 - p1)

        # If the z-component of the cross product is negative, swap p2 and p3 to ensure counter-clockwise order
        if cross_product[2] < 0:
            p2, p3 = p3, p2

        # Calculate the unit normal vectors for each edge
        edge1 = p2 - p1
        edge2 = p3 - p2
        edge3 = p1 - p3
        norm1 = self.unit_vector(self.perpendicular_vector(edge1))
        norm2 = self.unit_vector(self.perpendicular_vector(edge2))
        norm3 = self.unit_vector(self.perpendicular_vector(edge3))

        # Offset each vertex
        p1_new = p1 + norm1 * buffer_distance + norm3 * buffer_distance
        p2_new = p2 + norm1 * buffer_distance + norm2 * buffer_distance
        p3_new = p3 + norm2 * buffer_distance + norm3 * buffer_distance

        return p1_new, p2_new, p3_new

class BoundBox3D:
    def __init__(self, p1, p2, p3, xbuffer=0, ybuffer=0, zbuffer=0, color="b"):
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.p3 = np.array(p3)
        self.color = color
        self.box_min = np.min([self.p1, self.p2, self.p3], axis=0)
        self.box_max = np.max([self.p1, self.p2, self.p3], axis=0)

        # Add a small buffer to the box
        self.box_min[0] += -xbuffer
        self.box_max[0] += xbuffer
        
        self.box_min[1] += -ybuffer
        self.box_max[1] += ybuffer

        self.box_min[2] += -zbuffer
        self.box_max[2] += zbuffer

        self.points = [
            [self.box_min[0], self.box_min[1], self.box_min[2]],
            [self.box_min[0], self.box_min[1], self.box_max[2]],
            [self.box_min[0], self.box_max[1], self.box_min[2]],
            [self.box_min[0], self.box_max[1], self.box_max[2]],
            [self.box_max[0], self.box_min[1], self.box_min[2]],
            [self.box_max[0], self.box_min[1], self.box_max[2]],
            [self.box_max[0], self.box_max[1], self.box_min[2]],
            [self.box_max[0], self.box_max[1], self.box_max[2]]
        ]

    def plot(self, fig, ax,plot):
        # Define edges of the box
        x = [self.box_min[0], self.box_max[0]]
        y = [self.box_min[1], self.box_max[1]]
        z = [self.box_min[2], self.box_max[2]]

        # Create grid for each face and plot surfaces
        xx, yy = np.meshgrid(x, y)
        ax.plot_surface(xx, yy, np.full_like(xx, z[0]), color=self.color, alpha=0.1)
        ax.plot_surface(xx, yy, np.full_like(xx, z[1]), color=self.color, alpha=0.1)

        yy, zz = np.meshgrid(y, z)
        ax.plot_surface(np.full_like(yy, x[0]), yy, zz, color=self.color, alpha=0.1)
        ax.plot_surface(np.full_like(yy, x[1]), yy, zz, color=self.color, alpha=0.1)

        xx, zz = np.meshgrid(x, z)
        ax.plot_surface(xx, np.full_like(xx, y[0]), zz, color=self.color, alpha=0.1)
        ax.plot_surface(xx, np.full_like(xx, y[1]), zz, color=self.color, alpha=0.1)

        if plot:
            plt.show()

    def contains(self, point):
        return all(self.box_min[i] <= point[i] <= self.box_max[i] for i in range(len(point)))

class BoundCylinder:
    def __init__(self, center, radius, height):
        self.center = np.array(center)
        self.radius = radius
        self.height = height

    def plot(self, fig, ax, plot):
        x = np.linspace(self.center[0] - self.radius, self.center[0] + self.radius, 100)
        z = np.linspace(self.center[2] - self.radius, self.center[2] + self.radius, 100)
        X, Z = np.meshgrid(x, z)
        Y = np.sqrt(self.radius**2 - (X - self.center[0])**2) + self.center[1]
        ax.plot_surface(X, Y, Z, color="g", alpha=0.1, rstride=100, cstride=100)
        ax.plot_surface(X, -Y + 2*self.center[1], Z, color="g", alpha=0.1, rstride=100, cstride=100)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        if plot:
            plt.show()

    def contains(self, point):
        point = np.array(point)
        rad_dist = np.sqrt((point[0] - self.center[0])**2 + (point[1] - self.center[1])**2)

        if rad_dist < self.radius and self.center[2]-self.height/2 <= point[2] <= self.center[2] + self.height/2:
            return True 
        else:
            return False


# Initialize variables
arm_state = 0
num_flowers_pollinated = 0
time_since_last_pollination = 0.0

ns = ""
common_frame = "base"

pollination_distance = .01
step_size = .01

home_pose = Pose()
home_pose.position.x = 0.4 
home_pose.position.y =  0.0
home_pose.position.z = 0.0
home_pose.orientation.x = 0.0
home_pose.orientation.y = 0.0
home_pose.orientation.z = 0.0
home_pose.orientation.w = 1.0

backup_pose = copy.deepcopy(home_pose)

#                             x = 0.102,
reachable_space = BoundCylinder([0.03, 0, 1.3/2], 0.5, 1.3)

z_buffer = .20
xy_buffer = .08

arm_bounding_boxes = [BoundBox3D([-0.01, -0.01, -0.01], [-0.03, -0.03, -0.03], [-0.02, -0.02, -0.02], 0, 0, 0),
                      BoundBox3D([-0.01, -0.01, -0.01], [-0.03, -0.03, -0.03], [-0.02, -0.02, -0.02], 0, 0, 0),
                      BoundBox3D([-0.01, -0.01, -0.01], [-0.03, -0.03, -0.03], [-0.02, -0.02, -0.02], 0, 0, 0),
                      BoundBox3D([-0.01, -0.01, -0.01], [-0.03, -0.03, -0.03], [-0.02, -0.02, -0.02], 0, 0, 0),
                      BoundBox3D([-0.01, -0.01, -0.01], [-0.03, -0.03, -0.03], [-0.02, -0.02, -0.02], 0, 0, 0),
                      BoundBox3D([-0.01, -0.01, -0.01], [-0.03, -0.03, -0.03], [-0.02, -0.02, -0.02], 0, 0, 0)]

current_ee_pose = Pose()
target_pose = copy.deepcopy(home_pose)
told_pose = copy.deepcopy(home_pose)
previous_pose = copy.deepcopy(home_pose)

flowers = None

tf_listener = None

my_id = 0

# arm states 
ALL_FLOWERS_IN_COLLISION = 1 # red
GOING_TO_FLOWER = 2 # green
POLLINATING = 3 # yellow
BACKING_UP = 4  # blue
GOING_TO_TOLD_POSE = 5 # pink
NO_FLOWERS = 6 # blue-green
NO_FLOWERS_WITHIN_REACH = 7 # white

told_pose = None
done_pollinating = True
started_pollinating = False



#---------------------------------------------------- CALBACK FUNCTINS ------------------------------------------------
def unpollenated_flowers_callback(msg):
    global flowers

    flowers = msg.poses

def told_pose_callback(msg):
    global told_pose
    told_pose = msg
    
def ee_callback(msg):
    global done_pollinating
    done_pollinating = msg.data


#---------------------------------------------------- HELPER FUNCTIONS ------------------------------------------------

def calculate_distance(pose1, pose2):
    """Calculate the Euclidean distance between two poses."""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    dz = pose1.position.z - pose2.position.z
    return np.sqrt(dx*dx + dy*dy + dz*dz)

def get_tf(target_frame, source_frame, listener):
    tf_listener = listener
    try:
        # Wait for the transform to become available
        tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(secs=0.05))
        
        # Look up the transform
        (trans, rot) = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        pass

def get_manipulator_joint_feedback(listener):
    global current_ee_pose, backup_pose, my_id, arm_bounding_boxes, xy_buffer, z_buffer, common_frame

    # Assuming arm_ids is a list of all arm identifiers including the current arm's
    arm_ids = [1, 2, 3, 4, 5, 6]  # Example: You might want to dynamically generate this list

    for arm_id in arm_ids:
        # Skip updating the bounding box for the current arm here if you intend to do it separately
        if arm_id == my_id:
            continue

        # Construct namespace for the arm based on its ID
        arm_ns = "stickbug/arm{}/".format(arm_id)

        try:
            # Look up transform for each link in the arm
            box_pt, _ = get_tf(common_frame, arm_ns + 'box', listener)
            forearm_pt, _ = get_tf(common_frame, arm_ns + 'forearm', listener)
            end_pt, rot = get_tf(common_frame, arm_ns + 'end_link', listener)  # Assume rot needed only for end link
            
            # Update the bounding box for the arm
            box_pt[2] += .05
            arm_bounding_boxes[arm_id-1] = BoundBox3D(box_pt, forearm_pt, end_pt, z_buffer, xy_buffer, xy_buffer)
        except Exception as e:
             rospy.logwarn("Arm{} is not online or transform not available.".format(arm_id))

    # Now, separately update the bounding box and pose for the current arm
    box_pt, _ = get_tf(common_frame, ns + 'box', listener)
    forearm_pt, _ = get_tf(common_frame, ns + 'forearm', listener)
    end_pt, rot = get_tf(common_frame, ns + 'end_link', listener)

    # Update the bounding box for the current arm
    arm_bounding_boxes[my_id-1] = BoundBox3D(box_pt, forearm_pt, end_pt, z_buffer, xy_buffer, xy_buffer)

    # Update the current end effector pose
    current_ee_pose.position.x = end_pt[0]
    current_ee_pose.position.y = end_pt[1]
    current_ee_pose.position.z = end_pt[2]
    current_ee_pose.orientation.x = rot[0]
    current_ee_pose.orientation.y = rot[1]
    current_ee_pose.orientation.z = rot[2]
    current_ee_pose.orientation.w = rot[3]

    # Calculate the backup pose
    backup_pose = calculate_backup_pose(current_ee_pose, .05)

def calculate_backup_pose(ee_pose, distance_back=0.1):
    """
    Calculates a back up pose that is distance_back units directly opposite the orientation of the end effector in the XY plane.

    Parameters:
    ee_pose (Pose): The current pose of the end effector.
    distance_back (float): The distance to move back from the end effector in the XY plane.

    Returns:
    Pose: The calculated back up pose.
    """
    # Convert quaternion to a rotation matrix
    quaternion = (
        ee_pose.orientation.x,
        ee_pose.orientation.y,
        ee_pose.orientation.z,
        ee_pose.orientation.w
    )
    rotation_matrix = tft.quaternion_matrix(quaternion)
    
    # Calculate the backward direction vector in the end effector frame (assuming back is along the negative X-axis)
    backward_vector = [distance_back, 0, 0, 1]  # X, Y, Z, W (homogeneous coordinates)

    # Transform the backward vector by the rotation matrix to align with the end effector's orientation
    transformed_vector = rotation_matrix.dot(backward_vector)

    # Create the back up pose
    backup_pose = Pose()
    backup_pose.position.x = ee_pose.position.x - transformed_vector[0]
    backup_pose.position.y = ee_pose.position.y - transformed_vector[1]
    backup_pose.position.z = ee_pose.position.z - transformed_vector[2]
    backup_pose.orientation = ee_pose.orientation

    return backup_pose 

def sort_pose_list_by_distance(poses_list, reference_pose):
    """
    Sorts a PoseArray by the distance from a reference Pose.
    
    Parameters:
    pose_array (PoseArray): The array of poses to be sorted.
    reference_pose (Pose): The reference pose to calculate distances from.
    
    Returns:
    PoseArray: A new PoseArray with poses sorted by their distances from the reference pose.
    """
    # Convert PoseArray to a list of Pose for easier sorting
    poses_list
    
    # Sort the list of poses by distance from the reference pose
    sorted_poses_list = sorted(poses_list, key=lambda pose: calculate_distance(pose, reference_pose))
    
    return sorted_poses_list

def greedy_arm_controller():
    global reachable_space, flowers, current_ee_pose, target_pose, home_pose, num_flowers_pollinated, time_since_last_pollination
    global new_poll_flowers, my_id, arm_state, told_pose, started_pollinating, done_pollinating
    
    
    #=======================================================  GO TO TOLD POSITION ===============================
    if told_pose != None:
        if calculate_distance(current_ee_pose,told_pose) > .05:
            
            # if all zeros in the orientation of the told pose, then set it to the home pose
            if told_pose.orientation.x == 0 and told_pose.orientation.y == 0 and told_pose.orientation.z == 0 and told_pose.orientation.w == 0:
                # check to see if we can go home
                if not check_collision(home_pose):
                    told_pose = copy.deepcopy(home_pose)
                else:
                    # if not then just back up on the same z level
                    told_pose = copy.deepcopy(home_pose)
                    told_pose.position.z = current_ee_pose.position.z
                
                if calculate_distance(current_ee_pose,told_pose) < .05:
                    told_pose = None
                    time_since_last_pollination = 0.0

            # check that the told pose does not lead to a collision
            # if it does then freeze the arm
            if told_pose != None and not check_collision(told_pose):
                target_pose = copy.deepcopy(told_pose)
            elif told_pose != None:
                target_pose = copy.deepcopy(current_ee_pose)

            arm_state = GOING_TO_TOLD_POSE
        else:
            told_pose = None
            time_since_last_pollination = 0.0 # reset the timer 
    
    else:
    #=======================================================  POLLINATE FLOWER ===============================
        time_since_last_pollination += 1.0
        # if target is within pollination distance, pollinate
        if (calculate_distance(target_pose, current_ee_pose) < pollination_distance and arm_state == GOING_TO_FLOWER) or arm_state == POLLINATING:
            
            if not started_pollinating:
                rospy.loginfo(f"Arm {my_id} is Pollinating a flower!!!!!!!!!!!!!!!!!!!!!!!!!!")
                # trigger end effector to start pollinating 
                # publish end-effector on
                arm_state = POLLINATING
                started_pollinating = True
                done_pollinating = False
                
                
            elif done_pollinating:
                if len(flowers) > 0:
                    # incrment flower data -----------------------------------
                    num_flowers_pollinated += 1
                    time_since_last_pollination = 0.0
                    # remove the flower from the list
                    flowers = np.delete(flowers, 0, axis=0)
                
                    # add the flower to the newly pollinated list
                    new_poll_flowers = PoseArray()
                    new_poll_flowers.header.frame_id = f"{common_frame}{my_id}"
                    new_poll_flowers.poses.append(target_pose)
                
                started_pollinating = False
                # back up---------------------------------------------------
                arm_state = BACKING_UP 
                target_pose = copy.deepcopy(backup_pose)

      #=======================================================  SELECT CLOSEST FLOWER ===============================
        elif len(flowers) == 0:
            # if no flowers go home
            target_pose = copy.deepcopy(home_pose)
            rospy.loginfo(f"Arm {my_id} received no flowers")
            arm_state = NO_FLOWERS
        else:
            # remove flowers that are no longer reachable
            flowers = [flower for flower in flowers if reachable_space.contains([flower.position.x,flower.position.y,flower.position.z])]
            if len(flowers) == 0:
                # if no flowers go home
                target_pose = copy.deepcopy(home_pose)
                rospy.loginfo(f"Arm {my_id} No flowers within reach")
                arm_state = NO_FLOWERS_WITHIN_REACH
            else:
                # order flowers by closest to end effector
                sorted_flowers = sort_pose_list_by_distance(flowers, current_ee_pose)

                # pop the closest flower, check if the point is in collision, if so pop the next closest
                while len(sorted_flowers) > 0:
                    target_pose = sorted_flowers[0]
                
                    if not check_collision(target_pose):
                        rospy.loginfo(f"Arm {my_id} target position: " + str(target_pose.position))
                        arm_state = GOING_TO_FLOWER
                        break
                    else:
                        sorted_flowers = sorted_flowers[1:]
                        
      #=======================================================  NO FLOWER GO HOME ===============================
                # if no flowers are able to be reached, go to rest position
                if len(sorted_flowers) == 0:
                    target_pose = copy.deepcopy(home_pose)
                    rospy.loginfo(f"Arm {my_id} all flowers are in collision")
                    arm_state = ALL_FLOWERS_IN_COLLISION

                
    
def take_step_towards(current_pose, goal_pose, step_size):
    global current_ee_pose
    """
    Takes a step from current_ee_pose towards target_pose by step_size.
    
    Parameters:
    current_ee_pose (Pose): The current end-effector pose.
    target_pose (Pose): The target pose towards which to move.
    step_size (float): The step size towards the target pose.
    
    Returns:
    Pose: The new pose after taking a step.
    """

    # Calculate the direction vector from current to target
    direction = Pose()
    direction.position.x = goal_pose.position.x - current_ee_pose.position.x
    direction.position.y = goal_pose.position.y - current_ee_pose.position.y
    direction.position.z = goal_pose.position.z - current_ee_pose.position.z
    
    # Calculate the magnitude of the direction vector
    magnitude = (direction.position.x**2 + direction.position.y**2 + direction.position.z**2)**0.5
    
    # Normalize the direction vector
    if magnitude > 0:
        direction.position.x /= magnitude
        direction.position.y /= magnitude
        direction.position.z /= magnitude
    
    
    # Scale the direction vector by the step size
    step_vector = Pose()
    step_vector.position.x = direction.position.x * min(step_size,magnitude)
    step_vector.position.y = direction.position.y * min(step_size,magnitude)
    step_vector.position.z = direction.position.z * min(step_size,magnitude)

    
    # Calculate the new pose by adding the step vector to the current pose
    new_pose = Pose()
    new_pose.position.x = current_pose.position.x + step_vector.position.x
    new_pose.position.y = current_pose.position.y + step_vector.position.y
    new_pose.position.z = current_pose.position.z + step_vector.position.z
    
    # Assuming the orientation doesn't change for this small step
    new_pose.orientation = goal_pose.orientation
    
    return new_pose

def triangular_prism_to_cube_marker(prism, frame_id, marker_id):
    # Calculate the center of the triangle in the XY plane
    center_xy = np.mean([prism.p1[:2], prism.p2[:2], prism.p3[:2]], axis=0)

    # Calculate the height of the prism
    height = prism.z_max - prism.z_min

    # Calculate the approximate size of the cube by finding the max distance between the triangle vertices
    max_distance = max(np.linalg.norm(prism.p1 - prism.p2),
                       np.linalg.norm(prism.p2 - prism.p3),
                       np.linalg.norm(prism.p3 - prism.p1))

    # Add buffer to the cube size
    size = max_distance

    # Create the cube marker
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "triangular_prism_approximation"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = center_xy[0]
    marker.pose.position.y = center_xy[1]
    marker.pose.position.z = (prism.z_min + prism.z_max) / 2  # Center of the prism along Z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = height

    # Set the color of the marker
    if prism.color == "b":
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
    # Add other colors as needed
    marker.color.a = 0.1  # Set the alpha to make the cube semi-transparent

    marker.lifetime = rospy.Duration()  # 0 means the marker never auto-deletes

    return marker

def bounding_box_cube_to_marker(cube, frame_id, marker_id):
    # Calculate the center of the box
    center = (cube.box_min + cube.box_max) / 2.0

    # Calculate the scale of the box (dimensions)
    scale = cube.box_max - cube.box_min

    # Create the Marker message
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "bounding_box_cube"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    # Set the color of the marker based on the cube's color attribute
    if cube.color == "b":
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
    # Add other color mappings as needed
    marker.color.a = 0.5  # Set the alpha to make the cube semi-transparent

    marker.lifetime = rospy.Duration()  # 0 means the marker never auto-deletes

    return marker

def bound_cylinder_to_marker(cylinder, frame_id, marker_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "bound_cylinder"
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = cylinder.center[0]
    marker.pose.position.y = cylinder.center[1]
    marker.pose.position.z = cylinder.center[2]
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = 2 * cylinder.radius  # Diameter in x
    marker.scale.y = 2 * cylinder.radius  # Diameter in y
    marker.scale.z = cylinder.height  # Height
    marker.color.r = 0.0
    marker.color.g = 1.0  # Green color
    marker.color.b = 0.0
    marker.color.a = 0.1  # Semi-transparent
    marker.lifetime = rospy.Duration()

    return marker

def check_collision(pose_msg):
    global my_id, z_buffer, arm_bounding_boxes
    # Determine if the current arm is left or right
    is_right_arm = my_id % 2 == 0

    # Convert Pose message to a point (numpy array)
    point = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])

    # Filter arms on the same side but not the current arm
    same_side_arms = [arm for idx, arm in enumerate(arm_bounding_boxes) if (idx + 1) % 2 == my_id % 2 and idx + 1 != my_id]

    # Prepare z values for sorting and boundary checks
    z_points = [arm.box_min[2] for arm in same_side_arms] + [arm.box_max[2] for arm in same_side_arms]
    z_points.extend([point[2], 0, 1.3])
    z_points.sort()

    # Find current point's z in the sorted list and get the adjacent points
    my_index = z_points.index(point[2])
    my_lowerbound = z_points[my_index - 1]
    my_upperbound = z_points[my_index + 1]

    # Check if the target point's z is within the bounds considering the buffer
    if not my_lowerbound < point[2] < my_upperbound:
        return True

    # Target point bounding box check
    for idx, arm in enumerate(arm_bounding_boxes):
        # Exclude the current arm based on ID
        if idx + 1 != my_id:
            # Convert the bounding box to a marker and check if it contains the point
            if arm.contains(point):
                return True

    return False

    
    

#--------------------------------------------------- MAIN -------------------------------------------------------------

def arm_autonomy():

    global arm_state, num_flowers_pollinated, time_since_last_pollination,tf_listener, new_poll_flowers, target_pose
    global step_size, current_ee_pose, flowers, home_pose, told_pose, previous_pose, step_pose
    global my_id, ns, common_frame, started_pollinating, done_pollinating

    # initialize node ----------------------------------------------------------
    rospy.init_node('arm_autonomy')
    rate = rospy.Rate(1)

    # Extract arm number from namespace
    namespace = rospy.get_namespace()  # Get the current namespace
    my_id_match = re.search(r'arm(\d+)', namespace)
    if my_id_match:
        my_id = int(my_id_match.group(1))
    else:
        rospy.logwarn("Could not find arm number in namespace: {}".format(namespace))
        my_id = 0  # or set a default value

    ns = namespace[1:]


    # Construct the parameter name dynamically
    param_name = namespace + "common_frame"
    param_val = rospy.get_param(param_name, common_frame)
    if param_val  == common_frame:
        rospy.logwarn(f"The default value for 'common_frame' is being used: {common_frame}")
    common_frame = param_val
    print('Parameter value:',  common_frame )

    # Construct the parameter name dynamically
    param_name = namespace + "home_position"
    home_positions_params = rospy.get_param(param_name)

    home_pose.position.x = home_positions_params[0]
    home_pose.position.y = home_positions_params[1]
    home_pose.position.z = home_positions_params[2]
    home_pose.orientation.x = home_positions_params[3]
    home_pose.orientation.y = home_positions_params[4]
    home_pose.orientation.z = home_positions_params[5]
    home_pose.orientation.w = home_positions_params[6]

    print('home value:',  home_positions_params )

    # publisher and subscribers ------------------------------------------------
    # Subscribers
    rospy.Subscriber("unpollenated_flowers", PoseArray,unpollenated_flowers_callback)
    rospy.Subscriber("told_pose", Pose, told_pose_callback)
    rospy.Subscriber("ee_done", Bool, ee_callback)

    # Publishers
    arm_state_pub = rospy.Publisher("arm_state", Int64, queue_size=10)
    time_since_last_pollination_pub = rospy.Publisher("time_since_last_pollination", Float64, queue_size=10)
    number_pollinated_flowers_pub = rospy.Publisher("number_pollinated_flowers", Int64, queue_size=10)
    
    newly_pollinated_flower_pub = rospy.Publisher("/stickbug/newly_pollinated_flowers", PoseArray, queue_size=10)
    autonomy_pose_pub = rospy.Publisher("manipulator_goal", Pose, queue_size=10)
    ee_on = rospy.Publisher("end_effector_on", Bool, queue_size=10)

    hit_box_pub = rospy.Publisher("hit_box", Marker, queue_size=10)
    reached_box_pub = rospy.Publisher("reached_box", Marker, queue_size=10)

    
    # Initialize ------------------------------------------------------------------------
    tf_listener = tf.TransformListener()
    #get_manipulator_joint_feedback(listener=tf_listener)
    new_poll_flowers = PoseArray()
    tf_listener.waitForTransform(common_frame, ns + 'camera_frame', rospy.Time(0), rospy.Duration(secs=30))
    
    told_pose = copy.deepcopy(home_pose)
    arm_state = GOING_TO_TOLD_POSE

    
    # main loop -----------------------------------------------------------------
    loop_count = 0
    while not rospy.is_shutdown():
        
        # publish messages 
        # --------------------stat publishers
        arm_state_pub.publish(Int64(data=arm_state))
        time_since_last_pollination_pub.publish(Float64(data=time_since_last_pollination))
        number_pollinated_flowers_pub.publish(Int64(data=num_flowers_pollinated))
        #-------------------- arm control publishers

        if new_poll_flowers.poses:  # This checks if the list is not empty
            newly_pollinated_flower_pub.publish(new_poll_flowers)
            new_poll_flowers = PoseArray()
        
        # send start message to end effector
        if started_pollinating and not done_pollinating:
            ee_on.publish(Bool(data=started_pollinating))

        # publish reachability cylinder
        reached_box_pub.publish(bound_cylinder_to_marker(reachable_space, common_frame, my_id))

       
        if loop_count > 5:
            get_manipulator_joint_feedback(listener=tf_listener)
            cube_marker = bounding_box_cube_to_marker(arm_bounding_boxes[my_id-1], common_frame, my_id)
            greedy_arm_controller()
           # step_pose = take_step_towards(step_pose, target_pose, step_size)
            autonomy_pose_pub.publish(target_pose)
            hit_box_pub.publish(cube_marker)
            
        else:
            autonomy_pose_pub.publish(home_pose)
            step_pose = copy.deepcopy(home_pose)
            
        loop_count += 1
           
        
        # continue to loop -------------------------------------------------------
        rate.sleep()
    

if __name__ == '__main__':
    try:
      arm_autonomy()
    except rospy.ROSInterruptException:
       pass














