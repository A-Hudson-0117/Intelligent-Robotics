#!/usr/bin/env python
 
#import statements#
# We're going to need: rospy, tf_conversions, and tf2_ros.
import rospy
import math
import numpy as np
import tf_conversions 
import tf2_ros
 
# Here we are getting the tf broadcaster and message that will be used.
#from tf2_ros import TransformBroadcaster 	# removed duplicate
from geometry_msgs.msg import *
 
#Main function#
def main():
    # Initialize the node.
    rospy.init_node('tf2_broadcaster')
 
    # Make a new broadcaster and transform message.
    br = tf2_ros.TransformBroadcaster()

    # Set the transform frame ids.
    
    
    x = 2
    y = 3
    z = 0

    rad = math.radians(45)

    a = np.array(  [[math.cos(rad),-math.sin(rad), 0.0, x], 
                    [math.sin(rad), math.cos(rad), 0.0, y], 
                    [0.0          , 0.0          , 1.0, z], 
                    [0.0          , 0.0          , 0.0, 1.0]])
    matrix = np.asmatrix(a)
    t = convertMatToTf(matrix)

    t.header.frame_id = "world"
    t.child_frame_id = "basic"


    # part 3
    transf = convertMatToTf(translate(1,2,1))

    transf.header.frame_id = "world"
    transf.child_frame_id = "transformTest"

    #PART 4
    rotate45 = convertMatToTf(rotate(3.14/4,3.14/4,3.14/4))

    rotate45.header.frame_id = "world"
    rotate45.child_frame_id = "rotate45Test"

    rotate30 = convertMatToTf(rotate(3.14/3,3.14/3,3.14/3))

    rotate30.header.frame_id = "world"
    rotate30.child_frame_id = "rotate30Test"

    #part 5
    final = convertMatToTf(transform(0, 0, 2, 0, 0, 1.57))

    final.header.frame_id = "world"
    final.child_frame_id = "new2"


    # Wait a second
    rospy.sleep(1)

    # Send the transform message with the broadcaster
    br.sendTransform(t)
    br.sendTransform(transf)
    #br.sendTransform(rotate45)
    #br.sendTransform(rotate30)
    br.sendTransform(final)
    
    # Keeps broadcasting the transform
    rospy.spin()

def translate(x, y, z):
    a = np.array(  [[1.0 , 0.0 , 0.0 , x], 
                    [0.0 , 1.0 , 0.0 , y], 
                    [0.0 , 0.0 , 1.0 , z], 
                    [0.0 , 0.0 , 0.0 , 1.0]])
    matrix = np.asmatrix(a)
    return matrix

def rotateZ(theta):
    a = np.array(  [[math.cos(theta) ,-math.sin(theta) , 0.0 , 0], 
                    [math.sin(theta) , math.cos(theta) , 0.0 , 0], 
                    [0.0 , 0.0 , 1.0 , 0], 
                    [0.0 , 0.0 , 0.0 , 1]])
    matrix = np.asmatrix(a)
    return matrix

def rotateX(theta):
    a = np.array(  [[1.0 , 0.0 , 0.0 , 0.0], 
                    [0.0 , math.cos(theta) ,-math.sin(theta) , 0.0], 
                    [0.0 , math.sin(theta) , math.cos(theta) , 0.0], 
                    [0.0 , 0.0 , 0.0 , 1.0]])
    matrix = np.asmatrix(a)
    return matrix

def rotateY(theta):
    a = np.array(  [[math.cos(theta) , 0.0 , math.sin(theta) , 0.0], 
                    [0.0 , 1.0 , 0.0 , 0.0], 
                    [-math.sin(theta), 0.0 , math.cos(theta) , 0.0], 
                    [0.0 , 0.0 , 0.0 , 1.0]])
    matrix = np.asmatrix(a)
    return matrix

def rotate(rot_x,rot_y,rot_z):
    out = np.matmul(rotateX(rot_x), rotateY(rot_y))
    out = np.matmul(rotateZ(rot_z), out)
    return out

def transform(x, y, z, rot_x, rot_y, rot_z):
    return np.matmul(translate(x, y, z), rotate(rot_x, rot_y, rot_z))

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
            rot_y = math.pi / 2.0
            rot_x = rot_z + math.atan2(mat[0,1], mat[0,2])
        else:
            rot_y = -math.pi / 2.0
            rot_x = -rot_z + math.atan2(-mat[0,1], -mat[0,2])

    # Else, rot around y is not 90,-90
    else:
        rot_y = -math.asin(mat[2,0])
        #rot_y_2 = math.pi - rot_y
        
        rot_x = math.atan2(mat[2,1] / math.cos(rot_y), mat[2,2] / math.cos(rot_y))
        #rot_x_2 = math.atan2(mat[2][1] / math.cos(rot_y_2), mat[2][2] / math.cos(rot_y_2))
        
        rot_z = math.atan2( mat[1,0] / math.cos(rot_x), mat[0,0] / math.cos(rot_x))
        #rot_z_2 = math.atan2( mat[1][0] / math.cos(rot_x_2), mat[0][0] / math.cos(rot_x_2))

    # Get a Quaternion based on the euler angle rotations
    q = tf_conversions.transformations.quaternion_from_euler(rot_x, rot_y, rot_z)

    # Set rotation
    result.transform.rotation.x = q[0]
    result.transform.rotation.y = q[1]
    result.transform.rotation.z = q[2]
    result.transform.rotation.w = q[3]
    
    return result



#This is what will get executed#
if __name__ == '__main__':
    # Run the main function.
    main()
