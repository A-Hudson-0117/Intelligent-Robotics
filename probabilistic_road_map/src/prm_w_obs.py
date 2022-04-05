#!/usr/bin/env python3
import rospy
import numpy as np
import random
import tf_conversions
import tf2_ros
import math
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import *

import networkx as nx
import matplotlib.pyplot as plt

from probabilistic_road_map.msg import PRMPath
import traj_gen
import prm_obs_funcs as prm_obs

import fcl


distanceBetweenNodes = 2.5
NumberOfNodes = 250
pointScanResolution = 30

class Node:
    def __init__(self, value):
        self.value = value


def randomConfigurations():
      return (np.random.rand(3) * 2 - 1) * np.pi

  
def pubJointState(path):
    # print(thetas)

    pathMsg = PRMPath()
    for node in path:
       
        # Create JointState object
        j = JointState()
        j.header.stamp = rospy.Time.now()
        j.name = ['joint1', 'joint2', 'joint3']
        j.position = node.value
        
        pathMsg.qs.append(j)
 

    pub = rospy.Publisher('/prm_path', PRMPath, queue_size=10)
    rospy.sleep(1)
    pub.publish(pathMsg)



def localPlanner(c1, c2, obs):
    t = np.linspace(0, 1, pointScanResolution)
    for t_i in t :
        c = c2 * t_i + c1 * (1 - t_i)
        
        arm = prm_obs.getRobotCollOb(c)
                
        for o in obs:
            for a in arm:
                request = fcl.CollisionRequest()
                result = fcl.CollisionResult()

                ret = fcl.collide(o, a, request, result)
                if ret > 0:
                    return False
            
    return True
    


def checkForEdge(c1, c2, obs):
    # check configurations are "close" to each other
    near = np.sum(np.abs(c1 - c2)) < distanceBetweenNodes
    # consider collisions from localPlanner
    return near and localPlanner(c1, c2, obs)
    
    
#pub = rospy.Publisher("joint_states", JointState, queue_size=10)
rospy.init_node('prm')


def learningPhase(num, obs):
    graph = nx.Graph()
  
    for i in range(num):
        # add node to the graph
        n1 = Node(randomConfigurations())
        graph.add_node(n1)

        # connect possible nodes
        for n2 in (graph.nodes):
            # skip checking itself
            if n1 == n2:
                  continue
            # add edge based on function
            if checkForEdge(n1.value, n2.value, obs):
                  graph.add_edge(n1, n2)
    
    return graph


def queryPhase(sNode, gNode, graph, obs):
    for n in [sNode, gNode]:
        graph.add_node(n)
        # connect possible nodes
        for n1 in (graph.nodes):
          if n1 != n:
              # add edge based on function
              if checkForEdge(n1.value, n.value, obs):
                  graph.add_edge(n1, n)

    try:
        path = nx.shortest_path(graph, sNode, gNode)
        return path
    except nx.exception.NetworkXNoPath:
        return None
    

numObs = 20

def main():
    rospy.sleep(1) 
    obstacle_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)
    obstacles = [prm_obs.createObRand() for _ in range(numObs)]
    # print("collide")
    obs_collide = [ob.collObj for ob in obstacles]
    # print("Collide done")
    obstacles_msg = MarkerArray()
    obstacles_msg.markers = [prm_obs.buildRvizCube(ob, i) for i, ob in enumerate(obstacles)]
    
    print("Obstacles Added")

    rospy.sleep(1)
    
    obstacle_pub.publish(obstacles_msg)
    
    #rospy.sleep(1)
    
    print("Generating Graph")
    graph = learningPhase(NumberOfNodes, obs_collide)
    
    #nx.draw(graph)
    #plt.show()
    sNode =  Node(randomConfigurations())
    gNode =  Node(randomConfigurations())
    
    # rospy.sleep(1)
    print("Publishing Path")
    
    path = queryPhase(sNode, gNode, graph, obs_collide)
    print(path)
    
    rospy.sleep(1)
    
    if(path is not None):
        pubJointState(path)
    
#     # Loop through the nodes in the graph
#     for n in graph.nodes:
#         print(n)
#         print(n.value)
        
#         # Loop through all edges incident with node n
#         for e in graph.edges[n]:
#             print(e)
    
    
    print("done")

    rospy.spin()










if __name__ == "__main__":
    main()
