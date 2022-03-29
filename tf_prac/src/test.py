from pickle import FALSE
import numpy as np
import math as m
import rospy
import tf_conversions 
import tf2_ros
from geometry_msgs.msg import * 

  
  
def blarg():
    print("Hi")
  

def getT(alpha_iMinusOne, a_iMinusOne, d_i, theta_i):
    c_theta_i = m.cos(m.radians(theta_i))
    s_theta_i = m.sin(m.radians(theta_i))
    c_alpha_iMinusOne = m.cos(m.radians(alpha_iMinusOne))
    s_alpha_iMinusOne = m.sin(m.radians(alpha_iMinusOne))
    mat = np.array([
      [c_theta_i, -s_theta_i, 0, a_iMinusOne],
      [s_theta_i * c_alpha_iMinusOne, c_theta_i * c_alpha_iMinusOne, -s_alpha_iMinusOne, -s_alpha_iMinusOne * d_i],
      [s_theta_i * s_alpha_iMinusOne, c_theta_i * s_alpha_iMinusOne, c_alpha_iMinusOne, c_alpha_iMinusOne * d_i],
      [0, 0, 0, 1],
    ])
    matrix = np.asmatrix(mat)
    return matrix

def main():
    rospy.init_node("final_node_transform")
    t_0 = getT(0,       0,          0,          0)
    t_1 = getT(0,       0 ,         0.0892 ,    0)
    t_2 = getT(90 ,     0 ,         0 ,         0)
    t_3 = getT(0 ,     -0.425  ,    0 ,         0)
    t_4 = getT(0 ,     -0.39225,    0.1092 ,    0)
    t_5 = getT(90,      0 ,         0.0947 ,    0)
    t_6 = getT(-90,     0 ,         0.0823 ,    0)

    if( False ):
        t_cumulative = testpoint(t_0, t_1)
        t_cumulative = testpoint(t_cumulative, t_2)
        t_cumulative = testpoint(t_cumulative, t_3)
        t_cumulative = testpoint(t_cumulative, t_4)
        t_cumulative = testpoint(t_cumulative, t_5)
        t_cumulative = testpoint(t_cumulative, t_6)
    else:
        T = t_0 * t_1 * t_2 * t_3 * t_4 * t_5 * t_6 
    
        t = convertMatToTf(T)
        t.header.frame_id = "base"
        t.child_frame_id = "myEEFrame" #+ str(counter)
        t.header.stamp = rospy.get_rostime()
        print(T)
    
        rospy.sleep(1)
        br.sendTransform(t)



    rospy.spin()

br = tf2_ros.TransformBroadcaster()

    
def testpoint(preceeding, next):
    T_next = preceeding * next
    
    t = convertMatToTf(T_next)
    t.header.frame_id = "base"
    t.child_frame_id = "myEEFrame" #+ str(counter)
    t.header.stamp = rospy.get_rostime()
    print(T_next)
    
    rospy.sleep(1)
    br.sendTransform(t)
    rospy.sleep(2)
    return T_next
    
    
def convertMatToTf(mat):
    """
    Converts a numpy.mat into a geometry_msgs/TransformStamped object. These objects can be published to the tf hierarchy.

    Parameters:
        mat (numpy.mat): 4x4 matrix representing a pose

    Return:
        geometry_msgs/TransformStamped: An object representing the 4x4 matrix passed in.
    """
    result = TransformStamped()

    # Translation is set based on position vector in transformation matrix
    result.transform.translation.x = mat[0,3]
    result.transform.translation.y = mat[1,3]
    result.transform.translation.z = mat[2,3]

    ###########################################################
    # Get the rotation values around each axis
    ###########################################################
    
    # If rotation around y is 90 or -90
    if (mat[2,0] >= -1.01 and mat[2,0] <= -0.99) or (mat[2,0] >= 0.99 and mat[2,0] <= 1.01):

        # Set rot_z to anything, usually 0 is selected
        rot_z = 0.0
        if (mat[2,0] >= -1.01 and mat[2,0] <= -0.99):
            rot_y = m.pi / 2.0
            rot_x = rot_z + m.atan2(mat[0,1], mat[0,2])
        else:
            rot_y = -m.pi / 2.0
            rot_x = -rot_z + m.atan2(-mat[0,1], -mat[0,2])

    # Else, rot around y is not 90,-90
    else:
        rot_y = -m.asin(mat[2,0])
        #rot_y_2 = math.pi - rot_y
        
        rot_x = m.atan2(mat[2,1] / m.cos(rot_y), mat[2,2] / m.cos(rot_y))
        #rot_x_2 = math.atan2(mat[2][1] / math.cos(rot_y_2), mat[2][2] / math.cos(rot_y_2))
        
        rot_z = m.atan2( mat[1,0] / m.cos(rot_x), mat[0,0] / m.cos(rot_x))
        #rot_z_2 = math.atan2( mat[1][0] / math.cos(rot_x_2), mat[0][0] / math.cos(rot_x_2))

    # Get a Quaternion based on the euler angle rotations
    q = tf_conversions.transformations.quaternion_from_euler(rot_x, rot_y, rot_z)

    # Set rotation
    result.transform.rotation.x = q[0]
    result.transform.rotation.y = q[1]
    result.transform.rotation.z = q[2]
    result.transform.rotation.w = q[3]
    
    return result


if __name__ == "__main__":
    blarg()
    main()
