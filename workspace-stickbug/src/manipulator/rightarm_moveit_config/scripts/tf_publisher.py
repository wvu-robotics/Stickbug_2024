#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

class RobotTFPublisher:
    def __init__(self):
        rospy.init_node('robot_tf_publisher')
        self.br = tf.TransformBroadcaster()

        # Load URDF model
        self.urdf = URDF.from_parameter_server()

        # Subscribe to the joint states topic
        rospy.Subscriber('/arm0/manipulator_actual_joints', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        # Extract transformations from URDF
        for joint_state, joint_pose in zip(msg.name, msg.position):
            try:
                joint = self.urdf.joint_map[joint_state]
                rospy.loginfo("publishig joints %s", joint)
                if joint.parent and joint.child:
                    parent_link = joint.parent
                    child_link = joint.child

                    # Get the joint's transform
                    joint_transform = (joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2])
                    joint_rotation = tf.transformations.quaternion_from_euler(
                        joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]
                    )

                    # Broadcast the transform
                    self.br.sendTransform(joint_transform, joint_rotation, rospy.Time.now(), child_link, parent_link)

            except KeyError:
                rospy.logwarn("Joint %s not found in URDF. Skipping transformation calculation." % joint_state)

def main():
    robot_tf_publisher = RobotTFPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
