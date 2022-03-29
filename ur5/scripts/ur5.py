#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

rospy.init_node('ur5', anonymous=False)
pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)


def reset():
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    j.position = [0., 0., 0.0, 0.0, 0., 0.0]
    j.velocity = []
    j.effort = []

    pub_joints.publish(j)

