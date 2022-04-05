#!/usr/bin/env python3
import fcl
import rospy
import numpy
import random
import tf_conversions
import tf2_ros
from math import *


from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import *

class BoxOb:
 
    # posArray should be numpy.array. w,l,h are floats
    def __init__(self, w, l, h, posArray):
        self.w = w
        self.l = l
        self.h = h
        self.posArray = posArray
 

        self.collGeom = fcl.Box(w, l, h) # This should be a CollisionGeometry object from fcl
        self.collTf =  fcl.Transform(posArray) # This should be a Transform object from fcl
        self.collObj = fcl.CollisionObject(self.collGeom, self.collTf) # This should be a CollisionObject object from fcl
 
def createObRand():
    #print('In createOb')
 
    # Workspace bounds
    xmin = -5.0
    xmax = 5.0
    ymin = -5.0
    ymax = 5.0
    zmin = -2.0
    zmax = 8.0
    
    obMin = 0.33
    obMax = 1.5
    
    # Box params
    l = random.uniform(obMin, obMax)
    w = random.uniform(obMin, obMax)
    h = random.uniform(obMin, obMax)
 
    # Rejection sampling approach to get an obstacle center 
    # within the robot's workspace
    withinWorkspace = False
    while not withinWorkspace:
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        z = random.uniform(zmin, zmax)
 
        withinWorkspace = sqrt(x**2 + y**2 + z**2) < 5.0
 
    pos = numpy.array([x, y, z])
    
    obj = BoxOb(l,w,h,pos)
 
    #print('Exiting createOb')
    return obj
 
 
 
'''
boxOb should be type BoxOb. i should be an integer id for the obstacle.
'''
def buildRvizCube(boxOb, i):
    pointMarker = Marker()
    pointMarker.header.frame_id = 'base_link'
    pointMarker.header.stamp = rospy.Time(0)
    pointMarker.ns = ''
 
    pointMarker.id = 10+i
    pointMarker.type = 1
    pointMarker.action = 0
    
    pointMarker.scale.x = boxOb.w
    pointMarker.scale.y = boxOb.l
    pointMarker.scale.z = boxOb.h
 
    pointMarker.color.b = 1.0
    pointMarker.color.a = 1.0
    pointMarker.colors.append(pointMarker.color)
 
    pointMarker.pose.position.x = boxOb.posArray[0]
    pointMarker.pose.position.y = boxOb.posArray[1]
    pointMarker.pose.position.z = boxOb.posArray[2]
  
    return pointMarker
 
def translate(x, y, z):
    a = numpy.array(  [[1.0 , 0.0 , 0.0 , x], 
                    [0.0 , 1.0 , 0.0 , y], 
                    [0.0 , 0.0 , 1.0 , z], 
                    [0.0 , 0.0 , 0.0 , 1.0]])
    matrix = numpy.asmatrix(a)
    return matrix

def rotateZ(theta):
    a = numpy.array(  [[cos(theta) ,-sin(theta) , 0.0 , 0], 
                    [sin(theta) , cos(theta) , 0.0 , 0], 
                    [0.0 , 0.0 , 1.0 , 0], 
                    [0.0 , 0.0 , 0.0 , 1]])
    matrix = numpy.asmatrix(a)
    return matrix

def rotateX(theta):
    a = numpy.array(  [[1.0 , 0.0 , 0.0 , 0.0], 
                    [0.0 , cos(theta) ,-sin(theta) , 0.0], 
                    [0.0 , sin(theta) , cos(theta) , 0.0], 
                    [0.0 , 0.0 , 0.0 , 1.0]])
    matrix = numpy.asmatrix(a)
    return matrix

def rotateY(theta):
    a = numpy.array(  [[cos(theta) , 0.0 , sin(theta) , 0.0], 
                    [0.0 , 1.0 , 0.0 , 0.0], 
                    [-sin(theta), 0.0 , cos(theta) , 0.0], 
                    [0.0 , 0.0 , 0.0 , 1.0]])
    matrix = numpy.asmatrix(a)
    return matrix

 
def getRobotCollOb(q):
    #print('In getRobotCollOb')
 
    l1 = 3.0
    l2 = 3.0
    l3 = 2.0
 
    # Create capsules for robot's links
    link1 = fcl.Capsule(0.15, l1)
    link2 = fcl.Capsule(0.15, l2)
    link3 = fcl.Capsule(0.15, l3)
 
    '''
    Link 1
    '''
    # Rotation around z: theta1
    # Translation 1.5 on z to account for capsule origin in center
    R1 = rotateZ(q[0])
    TR1 = translate(0.0, 0.0, l1/2.0)
    TF1 = R1 * TR1
    TF1_FCL = fcl.Transform(numpy.asarray(R1[0:3,0:3]), TR1[0:3,3])
    ob1 = fcl.CollisionObject(link1, TF1_FCL)
    
    '''
    Link 2
    '''
    # Rotation around y: 90 - theta2 (when theta2=0, it's rotated 90 degrees from being vertical. It's relative to world y-axis which is opposite of the joint axis, so 90 degrees for the joint would point vertical up not vertical down)
    # Translation: 1.5 on z to go up from link 1
    l2RotY = (pi / 2.0) - q[1]
    R2 = rotateY(l2RotY)
    TR2 = translate(0.0, 0.0, l2/2.0)
 
    # Transform for link 2
    TF2 = R2 * TR2
 
    # Now account for link 1 by transforming by link 1
    # Add in another translate to account for other half of link 1
    TF2 = TF1 * translate(0.0, 0.0, l1/2.0) * TF2
 
    # Create FCL objects
    TF2_FCL = fcl.Transform(numpy.asarray(TF2[0:3,0:3]), TF2[0:3,3])
    ob2 = fcl.CollisionObject(link2, TF2_FCL)
    
 
    '''
    Link 3
    '''
    # Rotation around y: -theta3. Why negative? B/c it's relative to world frame, and the joint axis points in the opposite direction of the world y-axis.
    # Translation: 1 on z to set capsule origin correctly
    l3RotY = -q[2]
    R3 = rotateY(l3RotY)
    TR3 = translate(0.0, 0.0, l3/2.0)
    TF3 = R3 * TR3
 
    # Now account for link 2 by transforming by link 2
    # Add in another translate to account for other half of link 1
    TF3 = TF2 * translate(0.0, 0.0, l2/2.0) * TF3
 
    # Create FCL objects
    TF3_FCL = fcl.Transform(numpy.asarray(TF3[0:3,0:3]), TF3[0:3,3])
    ob3 = fcl.CollisionObject(link3, TF3_FCL)
 
    return [ob1, ob2, ob3]
