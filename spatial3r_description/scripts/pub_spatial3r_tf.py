#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped



pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)



def pubJointState():
    #print('In pubJointState')
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['joint1', 'joint2', 'joint3']
    j.position = [0.0, 0.0, 0.0]
    j.velocity = []
    j.effort = []

    pub_joints.publish(j)
    #print('Exiting pubJointState')


def pubTFs():
    #print('In pubTFs')

    br = tf2_ros.TransformBroadcaster()

    # When everything is at 0...
    

    # link 1 Parent=base_link
    t_link1 = TransformStamped()
    t_link1.header.stamp = rospy.Time.now()
    t_link1.header.frame_id = 'base_link'
    t_link1.child_frame_id = 'link1'
    t_link1.transform.translation.x = 0
    t_link1.transform.translation.y = 0
    t_link1.transform.translation.z = 0
    
    q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    t_link1.transform.rotation.x = q[0]
    t_link1.transform.rotation.y = q[1]
    t_link1.transform.rotation.z = q[2]
    t_link1.transform.rotation.w = q[3]

    # link 2, Parent=link 1
    t_link2 = TransformStamped()
    t_link2.header.stamp = rospy.Time.now()
    t_link2.header.frame_id = 'link1'
    t_link2.child_frame_id = 'link2'
    t_link2.transform.translation.x = 0
    t_link2.transform.translation.y = 0
    t_link2.transform.translation.z = 3
    
    q = tf_conversions.transformations.quaternion_from_euler(0,0,1.5708)
    t_link2.transform.rotation.x = q[0]
    t_link2.transform.rotation.y = q[1]
    t_link2.transform.rotation.z = q[2]
    t_link2.transform.rotation.w = q[3]


    # link 3, Parent=link 2
    t_link3 = TransformStamped()
    t_link3.header.stamp = rospy.Time.now()
    t_link3.header.frame_id = 'link2'
    t_link3.child_frame_id = 'link3'
    t_link3.transform.translation.x = 3
    t_link3.transform.translation.y = 0
    t_link3.transform.translation.z = 0
    
    q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    t_link3.transform.rotation.x = q[0]
    t_link3.transform.rotation.y = q[1]
    t_link3.transform.rotation.z = q[2]
    t_link3.transform.rotation.w = q[3]


    # endEffector, Parent=link 3
    t_ee = TransformStamped()
    t_ee.header.stamp = rospy.Time.now()
    t_ee.header.frame_id = 'link3'
    t_ee.child_frame_id = 'endEffector'
    t_ee.transform.translation.x = 2
    t_ee.transform.translation.y = 0
    t_ee.transform.translation.z = 0
    
    q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    t_ee.transform.rotation.x = q[0]
    t_ee.transform.rotation.y = q[1]
    t_ee.transform.rotation.z = q[2]
    t_ee.transform.rotation.w = q[3]


    # Send the transforms
    br.sendTransform(t_link1)
    br.sendTransform(t_link2)
    br.sendTransform(t_link3)
    br.sendTransform(t_ee)
    
    

    #print('Exiting pubTFs')


def main():
    print('In main')
    
    rospy.init_node('pub_spatial3r_tf', anonymous=False)
    
    rospy.sleep(0.5)
    pubTFs()
    pubJointState()
    

    print('Exiting normally')


if __name__ == '__main__':
    main()