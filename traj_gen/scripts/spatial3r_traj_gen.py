import rospy
import numpy as np
from geometry_msgs.msg import *
from std_msgs.msg import *

# IK
import tf_conversions
import tf2_ros
from math import sin, cos, atan2, sqrt, pow, acos, pi, fmod
from sensor_msgs.msg import *


# Authors:  Adam Hudson,  Johnathon Mendenhall,  Yashiya Minor  


l1 = 3.0
l2 = 3.0
l3 = 2.0




# Publisher to use in the pubJointState function
pub_joints = rospy.Publisher("joint_states", JointState, queue_size=10)
 
def main():
    rospy.init_node('trajectory_generator')

    rot = Quaternion(0,0,0,1)
    start = Point(5.0, 0.0, 3.0)
    startPose = Pose(start, rot)
    p1 = Point(2.6481, 1.5281, 6.8881)
    p2 = Point(-1.3655, 3.8613, 4.2754)
    p3 = Point(-2.4241, -2.753, 3.2393)
    p1Pose = Pose(p1, rot)
    p2Pose = Pose(p2, rot)
    p3Pose = Pose(p3, rot)
  
    path = [startPose, p1Pose, p2Pose, p3Pose, startPose]
  
    ik_sols = []
  
      # find closest chain of ik solutions on path
    prev_ik_sol = None
    for pose in path:
        sols = IK(pose)
        print(sols)
        # no valid solutions for this
        if sols is None:
            continue
        if prev_ik_sol is None:
            # use a specific solution on first pose
            sol = sols[0]
        else:
            # find closest ik solution to previous solution
            sorted_sols = sorted(sols, key=lambda sol: np.sum(np.abs(np.array(sol) - np.array(prev_ik_sol))))
            sol = sorted_sols[0]
      	
        ik_sols.append(sol)
        # store this solution so next knot can find closest ik solution to it
        prev_ik_sol = sol

    T = 3
    resolution = 0.1
    
    all_sols = [[], [], []]
    
    print(ik_sols)
    
      # interpolate between knots
    for sol0, sol1 in zip(ik_sols[:-1], ik_sols[1:]):
        # sol0 = starting solution
        # sol1 = goal solution
        # for this portion of the path
        
        joints_t = []
        for i, (joint0, joint1) in enumerate(zip(sol0, sol1)):
            cc = cubCoef(joint0, joint1, T)
            # interpolated joint angles from joint0 to joint1
            joint_t = trajec(T, resolution, cc)[0]
            all_sols[i] += joint_t

    
    
    
    # publish the points
    sendTrajToRviz(all_sols, 0.05)
    
    
    
    

  
# step 6
'''
Sends the trajectory positions to rviz at the specified time resolution
 
Parameters:
    allPos (list): A list of all the positions for each joint. 
                   The order of indices should be [joint][position].
    resolution (float): The elapsed time between each trajectory point
Returns:
    None
'''
def sendTrajToRviz(allPos, resolution):
    rate = rospy.Rate(10)
    T = len(allPos[0]) * resolution

    tNow = rospy.Time.now()
    tEnd = tNow + rospy.Duration(T)

    while rospy.Time.now() < tEnd and not rospy.is_shutdown():
        thetas = []
        index = int((rospy.Time.now() - tNow).to_sec() / resolution)
        for i in range(len(allPos)):
            thetas.append(allPos[i][index])
        
        pubJointState(thetas)
        rate.sleep()  


  
def pubJointState(thetas):
    '''
    Converts a list of joint values to a JointState object, and published the object to topic "joint_states"
    Parameters:
        thetas (list): a list with three joint values
    
    Returns:
        None
    '''

    # Create JointState object
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['joint1', 'joint2', 'joint3']
    j.position = thetas
    j.velocity = []
    j.effort = []

    # Publish it
    pub_joints.publish(j)
    
    

# step 4
def wrapPi(angs):
    for i in range(len(angs)):            
        if angs[i] < 0:
            angs[i] = angs[i] % pi
    
# step prep
def findAngleBetweenAngles(a, b):    
    result = 0
    difference = b - a
    
    if difference > pi:
        difference = fmod(difference, pi)
        result = difference - pi
        
    elif(difference < - pi):
          result = difference + (2.0*pi)

    else:
        result = difference

    return result


# step 1
def cubCoef(qs, qg, T):
    delta_q = findAngleBetweenAngles(qs, qg)
    
    a0 = qs
    a1 = 0
    a2 = 3 / T ** 2 * delta_q
    a3 = -2 / T ** 3 * delta_q
    
    return a0, a1, a2, a3
    
    
# step 2
def trajec(T, resolution, cc):
    t = np.linspace(0, T, T / resolution)
      
    a0, a1, a2, a3 = cc
  
    pos = [a0 + a1 * ti + a2 * ti ** 2 + a3 * ti ** 3 for ti in t]
    vel = [a1 + 2 * a2 * ti + 3 * a3 * ti ** 2 for ti in t]
    acc = [2 * a2 + 6 * a3 * ti for ti in t]
    
    return pos, vel, acc

  

  
  
  
  
  
  
  
# step 5
pub_joints = rospy.Publisher("joint_states", JointState, queue_size=10)

def pubJointState(thetas):
    '''
    Converts a list of joint values to a JointState object, and published the object to topic "joint_states"
    Parameters:
        thetas (list): a list with three joint values
    
    Returns:
        None
    '''

    # Create JointState object
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['joint1', 'joint2', 'joint3']
    j.position = thetas
    j.velocity = []
    j.effort = []

    # Publish it
    pub_joints.publish(j)


def findDistanceBetweenAngles(a, b):
    '''
    Get the smallest orientation difference in range [-pi,pi] between two angles 
    Parameters:
        a (double): an angle
        b (double): an angle
    
    Returns:
        double: smallest orientation difference in range [-pi,pi]
    '''
    result = 0
    difference = b - a
    
    if difference > pi:
      difference = fmod(difference, pi)
      result = difference - pi

    elif(difference < -pi):
      result = difference + (2*pi)

    else:
      result = difference

    return result



def displaceAngle(a1, a2):
    '''
    Displace an orientation by an angle and stay within [-pi,pi] range
    Parameters:
        a1 (double): an angle
        a2 (double): an angle
    
    Returns:
        double: The resulting angle in range [-pi,pi] after displacing a1 by a2
    '''
    a2 = a2 % (2.0*pi)

    if a2 > pi:
        a2 = (a2 % pi) - pi

    return findDistanceBetweenAngles(-a1,a2)

    
def solveTheta2And3(r, z):
    '''
    Solves for theta 2 and 3. There are two solutions.

    Parameters:
        r (double): signed length of vector from the origin to the end-effector position on the xy plane
        z (double): z value of the end-effector position

    Returns:
        list: 2D list in the form [[theta2A, theta3A], [theta2B, theta3B]]
    '''

    print('In solveTheta23')

    #******************************************************
    # theta2 + theta3
    #******************************************************
    A = -2.0*l3*r
    B = -2.0*l3*(z - l1)
    C = pow(r,2) + pow(z-l1,2) + pow(l3,2) - pow(l2,2)

    # Compute psi angle
    psi = atan2(B,A)

    # Check if solution exists based on domain of acos before computing theta23 solutions
    if -C/(sqrt(pow(A,2) + pow(B,2))) < -1 or -C/(sqrt(pow(A,2) + pow(B,2))) > 1:
        print('No solution exists!')
        return []

    # Two pairs of values of theta2 and theta3 based on +-acos
    theta23_a = acos( -C/(sqrt(pow(A,2) + pow(B,2))) ) + psi
    theta23_b = -acos( -C/(sqrt(pow(A,2) + pow(B,2))) ) + psi
    

    #******************************************************
    # theta2
    #******************************************************
    theta2_a = atan2( z-l1 - (l3*sin(theta23_a)), r - (l3*cos(theta23_a)) )
    theta2_b = atan2( z-l1 - (l3*sin(theta23_b)), r - (l3*cos(theta23_b)) )


    # STUDENT
    # Solve for theta3 solutions below
    #******************************************************
    # theta3
    #******************************************************
    
    theta3_a = findDistanceBetweenAngles(theta2_a, theta23_a)
    theta3_b = findDistanceBetweenAngles(theta2_b, theta23_b)

    print('Exiting solveTheta23')
    return [[(theta2_a), (theta3_a)], [(theta2_b), (theta3_b)]]


def IK(eePose):
    '''
    Computes IK solutions for a given end-effector pose.

    Parameters:
        eePose (Pose): the pose of the end-effector

    Returns:
        list: The 4 solutions for IK, or None if no solution exists
    '''
    print("In IK")
    x = eePose.position.x
    y = eePose.position.y
    z = eePose.position.z

    # STUDENT
    # Solve for theta 1 below. There are two solutions.
    #******************************************************
    # theta1
    # theta 1 is based on an overhead view.
    #******************************************************
    if (x ** 2 + y ** 2 + (z - l1) ** 2) ** 0.5 > l2 + l3:
        print("geometry fail")
        return None
    
    theta1_a = atan2(y, x)
    theta1_b = theta1_a - pi






    # STUDENT Uncomment the lines below and fill in the parameters
    r = (x ** 2 + y ** 2) ** 0.5
    sols23a = solveTheta2And3(r, z)
    sols23b = solveTheta2And3(-r, z)
  

    # STUDENT
    # Create code that creates the list of solutions (if solutions exist) and return it
    # Return None if no solutions exist
    if len(sols23a) > 1:
        sol1 = [theta1_a, *sols23a[0]]
        sol2 = [theta1_a, *sols23a[1]]
        sol3 = [theta1_b, *sols23b[0]]
        sol4 = [theta1_b, *sols23b[1]]
        return [sol1, sol2, sol3, sol4]
    else:
        return None


  
  
  
  
  
  

  
  
if __name__ == "__main__":
    main()
  
  
  

