#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped



pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)


# j is a list of joint angles
def sendJointAngles(j):
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    j.position = j
    j.velocity = []
    j.effort = []


def pubJointState():
    print('In pubJointState')
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    j.position = [0.785, 0.785, 0.0, 0.0, 0.785, 0.0]
    j.velocity = []
    j.effort = []

    #pub_joints.publish(j)
    print('Exiting pubJointState')


def pubTFs(timerEvent):
    print('In pubTFs')

    br = tf2_ros.TransformBroadcaster()

    # When everything is at 0...
    

    # base_link Parent=base
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = 'base_link'
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    
    q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # world, Parent=
    t_world = TransformStamped()
    t_world.header.stamp = rospy.Time.now()
    t_world.header.frame_id = ''
    t_world.child_frame_id = 'world'
    t_world.transform.translation.x = 0
    t_world.transform.translation.y = 0
    t_world.transform.translation.z = 0
    
    q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    t_world.transform.rotation.x = q[0]
    t_world.transform.rotation.y = q[1]
    t_world.transform.rotation.z = q[2]
    t_world.transform.rotation.w = q[3]
    

    # ee_link, Parent=wrist_3_link
    t_ee_link = TransformStamped()
    t_ee_link.header.stamp = rospy.Time.now()
    t_ee_link.header.frame_id = 'wrist_3_link'
    t_ee_link.child_frame_id = 'ee_link'
    t_ee_link.transform.translation.x = 0
    t_ee_link.transform.translation.y = 0.0823
    t_ee_link.transform.translation.z = 0
    
    # How to get the quaternion values?
    t_ee_link.transform.rotation.x = 0
    t_ee_link.transform.rotation.y = 0
    t_ee_link.transform.rotation.z = 0.707
    t_ee_link.transform.rotation.w = 0.707
    euler_ee = tf_conversions.transformations.euler_from_quaternion([t_ee_link.transform.rotation.x, t_ee_link.transform.rotation.y, t_ee_link.transform.rotation.z, t_ee_link.transform.rotation.w])
    print('ee_link rotation euler angles: %s', euler_ee)


    # tool0, Parent=wrist_3_link
    t_tool0 = TransformStamped()
    t_tool0.header.stamp = rospy.Time.now()
    t_tool0.header.frame_id = 'wrist_3_link'
    t_tool0.child_frame_id = 'tool0'
    t_tool0.transform.translation.x = 0
    t_tool0.transform.translation.y = 0.0823
    t_tool0.transform.translation.z = 0
    
    t_tool0.transform.rotation.x = -0.707
    t_tool0.transform.rotation.y = 0
    t_tool0.transform.rotation.z = 0
    t_tool0.transform.rotation.w = 0.707
    tool0_ee = tf_conversions.transformations.euler_from_quaternion([t_tool0.transform.rotation.x, t_tool0.transform.rotation.y, t_tool0.transform.rotation.z, t_tool0.transform.rotation.w])
    print('t_tool0 rotation euler angles: %s', tool0_ee)

    # wrist_3_link, Parent=wrist_2_link
    t_wrist_3_link = TransformStamped()
    t_wrist_3_link.header.stamp = rospy.Time.now()
    t_wrist_3_link.header.frame_id = 'wrist_2_link'
    t_wrist_3_link.child_frame_id = 'wrist_3_link'
    t_wrist_3_link.transform.translation.x = 0
    t_wrist_3_link.transform.translation.y = 0
    t_wrist_3_link.transform.translation.z = 0.0946
    
    t_wrist_3_link.transform.rotation.x = 0
    t_wrist_3_link.transform.rotation.y = 0
    t_wrist_3_link.transform.rotation.z = 0
    t_wrist_3_link.transform.rotation.w = 1
    wrist3_euler = tf_conversions.transformations.euler_from_quaternion([t_wrist_3_link.transform.rotation.x, t_wrist_3_link.transform.rotation.y, t_wrist_3_link.transform.rotation.z, t_wrist_3_link.transform.rotation.w])
    print('t_tool0 rotation euler angles: %s', wrist3_euler)

    # forearm_link, Parent=upper_arm_link
    t_forearm_link = TransformStamped()
    t_forearm_link.header.stamp = rospy.Time.now()
    t_forearm_link.header.frame_id = 'upper_arm_link'
    t_forearm_link.child_frame_id = 'forearm_link'
    t_forearm_link.transform.translation.x = 0
    t_forearm_link.transform.translation.y = -0.1197
    t_forearm_link.transform.translation.z = 0.425
    
    t_forearm_link.transform.rotation.x = 0
    t_forearm_link.transform.rotation.y = 0
    t_forearm_link.transform.rotation.z = 0
    t_forearm_link.transform.rotation.w = 1
    forearm_euler = tf_conversions.transformations.euler_from_quaternion([t_forearm_link.transform.rotation.x, t_forearm_link.transform.rotation.y, t_forearm_link.transform.rotation.z, t_forearm_link.transform.rotation.w])
    print('forearm rotation euler angles: %s', forearm_euler)

    # shoulder_link, Parent=base_link
    t_shoulder_link = TransformStamped()
    t_shoulder_link.header.stamp = rospy.Time.now()
    t_shoulder_link.header.frame_id = 'base_link'
    t_shoulder_link.child_frame_id = 'shoulder_link'
    t_shoulder_link.transform.translation.x = 0
    t_shoulder_link.transform.translation.y = 0
    t_shoulder_link.transform.translation.z = 0.089
    
    t_shoulder_link.transform.rotation.x = 0
    t_shoulder_link.transform.rotation.y = 0
    t_shoulder_link.transform.rotation.z = 0
    t_shoulder_link.transform.rotation.w = 1
    shoulder_euler = tf_conversions.transformations.euler_from_quaternion([t_shoulder_link.transform.rotation.x, t_shoulder_link.transform.rotation.y, t_shoulder_link.transform.rotation.z, t_shoulder_link.transform.rotation.w])
    print('forearm rotation euler angles: %s', shoulder_euler)


    # upper_arm_link, Parent=shoulder_link
    t_upper_arm_link = TransformStamped()
    t_upper_arm_link.header.stamp = rospy.Time.now()
    t_upper_arm_link.header.frame_id = 'shoulder_link'
    t_upper_arm_link.child_frame_id = 'upper_arm_link'
    t_upper_arm_link.transform.translation.x = 0
    t_upper_arm_link.transform.translation.y = 0.135
    t_upper_arm_link.transform.translation.z = 0
    
    t_upper_arm_link.transform.rotation.x = 0
    t_upper_arm_link.transform.rotation.y = 0.707
    t_upper_arm_link.transform.rotation.z = 0
    t_upper_arm_link.transform.rotation.w = 0.707
    upper_arm_euler = tf_conversions.transformations.euler_from_quaternion([t_upper_arm_link.transform.rotation.x, t_upper_arm_link.transform.rotation.y, t_upper_arm_link.transform.rotation.z, t_upper_arm_link.transform.rotation.w])
    print('upper_arm rotation euler angles: %s', upper_arm_euler)


    # wrist_1_link, Parent=forearm_link
    t_wrist_1_link = TransformStamped()
    t_wrist_1_link.header.stamp = rospy.Time.now()
    t_wrist_1_link.header.frame_id = 'forearm_link'
    t_wrist_1_link.child_frame_id = 'wrist_1_link'
    t_wrist_1_link.transform.translation.x = 0
    t_wrist_1_link.transform.translation.y = 0
    t_wrist_1_link.transform.translation.z = 0.392
    
    t_wrist_1_link.transform.rotation.x = 0
    t_wrist_1_link.transform.rotation.y = 0.707
    t_wrist_1_link.transform.rotation.z = 0
    t_wrist_1_link.transform.rotation.w = 0.707
    wrist_1_euler = tf_conversions.transformations.euler_from_quaternion([t_wrist_1_link.transform.rotation.x, t_wrist_1_link.transform.rotation.y, t_wrist_1_link.transform.rotation.z, t_wrist_1_link.transform.rotation.w])
    print('upper_arm rotation euler angles: %s', wrist_1_euler)


    # wrist_2_link, Parent=wrist_1_link
    t_wrist_2_link = TransformStamped()
    t_wrist_2_link.header.stamp = rospy.Time.now()
    t_wrist_2_link.header.frame_id = 'wrist_1_link'
    t_wrist_2_link.child_frame_id = 'wrist_2_link'
    t_wrist_2_link.transform.translation.x = 0
    t_wrist_2_link.transform.translation.y = 0.093
    t_wrist_2_link.transform.translation.z = 0
    
    t_wrist_2_link.transform.rotation.x = 0
    t_wrist_2_link.transform.rotation.y = 0
    t_wrist_2_link.transform.rotation.z = 0
    t_wrist_2_link.transform.rotation.w = 1
    wrist_2_euler = tf_conversions.transformations.euler_from_quaternion([t_wrist_2_link.transform.rotation.x, t_wrist_2_link.transform.rotation.y, t_wrist_2_link.transform.rotation.z, t_wrist_2_link.transform.rotation.w])
    print('upper_arm rotation euler angles: %s', wrist_2_euler)


    # Send the transforms
    br.sendTransform(t)
    br.sendTransform(t_wrist_1_link)
    br.sendTransform(t_wrist_2_link)
    br.sendTransform(t_wrist_3_link)
    br.sendTransform(t_ee_link)
    br.sendTransform(t_forearm_link)
    br.sendTransform(t_tool0)
    #br.sendTransform(t_world)
    br.sendTransform(t_upper_arm_link)
    br.sendTransform(t_shoulder_link)
    

    pubJointState()

    print('Exiting pubTFs')
def main():
    print('In main')
    
    rospy.init_node('pubTFs', anonymous=False)
    
    #pubJointState()
    print('Starting timer')
    rospy.Timer(rospy.Duration(0.2), pubTFs)
    rospy.spin()
    

    print('Exiting normally')


if __name__ == '__main__':
    main()