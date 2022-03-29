#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
from math import sin, cos, atan2, sqrt, pow, acos, pi, fmod
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import numpy as np

l1 = 3.0
l2 = 3.0
l3 = 2.0

# Publisher to use in the pubJointState function
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



      
def random_sphere_point(cx, cy, cz, r):
    v = np.random.rand(3) * 2 - 1
    v /= np.linalg.norm(v)
    return np.array([cx, cy, cz]) + v * r

  
def main():
    print('In main')
    
    rospy.init_node('spatial3r_ik', anonymous=False)
    rospy.sleep(0.5)
    
    # generate random points on spheres
    inner_radius = l2 - l3
    inner_boundary_points_no = [random_sphere_point(0, 0, l1, inner_radius - 0.01) for _ in range(200)]
    inner_boundary_points_yes = [random_sphere_point(0, 0, l1, inner_radius + 0.01) for _ in range(200)]
    outer_radius = l2 + l3
    outer_boundary_points_no = [random_sphere_point(0, 0, l1, outer_radius + 0.01) for _ in range(200)]
    outer_boundary_points_yes = [random_sphere_point(0, 0, l1, outer_radius - 0.01) for _ in range(200)]
    
    # combine and give expected test result
    pts = [
        #*inner_boundary_points_no,
        #*outer_boundary_points_no,
        #*inner_boundary_points_yes,
        *outer_boundary_points_yes,
    ]
    has_solutions = [   #*(False for _ in inner_boundary_points_no),
                        #*(False for _ in outer_boundary_points_no),
                        #*(True for _ in inner_boundary_points_yes),
                        *(True for _ in outer_boundary_points_yes),
    ]
    
    # try test cases
    for pt, pt_has_solutions in zip(pts, has_solutions):
      
      # Test case end effector position
        p = Pose()
        p.position.x = pt[0]
        p.position.y = pt[1]
        p.position.z = pt[2]
      
        print("Goal:\n" + str(p) + "\n")
      
        sols = IK(p)

        correct = (sols is not None) == pt_has_solutions
      
      # If solutions exist, then publish the joint states in Rviz
        if sols is None:
            print("No Solution")
        elif len(sols) > 0:
            for i,s in enumerate(sols):
                print('Solution %s: %s' % (i,s))

            # STUDENT
            # Choose one of the solutions from the list to visualize
            q = sols[ 1 ]

            # Publish the joint values to Rviz
            pubJointState(q)

            # Check position with FK
            x = cos(q[0]) * (l2 * cos(q[1]) + l3*cos(q[1]+q[2]))
            y = sin(q[0]) * (l2 * cos(q[1]) + l3*cos(q[1]+q[2]))
            z = l1 + l2*sin(q[1]) + l3*sin(q[1]+q[2])
            #print('FK: [%s, %s, %s]' % (x, y, z))
        else:
            #print("FAILURE")
            return -1000000000000000000000000

      #print('Exiting normally')
        print("\n\n\n")




if __name__ == "__main__":
    main()