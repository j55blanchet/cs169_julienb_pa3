#!/usr/bin/env python
"""
    pa3optimizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that constucts a pose-graph from suscribed ros topics
    and then performs bundle optimization on the graph. Designed specifically for a 1D
    localization scenario and only considers a single landmark - the front-facing scan
"""

import sys
import rospy
import g2o
import numpy as np

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from posegraph import PoseGraph

WALL_VERTEX_ID = sys.maxint - 1

class PA3Optimizer:

    def __init__(self):
        rospy.init_node("graphoptimizer")

        self.posegraph = PoseGraph()

        rospy.Subscriber("pose", PoseStamped, self.on_pose, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)

        self.vertex_seq = 0
        self.prev_pose = None
    
    def on_pose(self, posemsg):
        """Handles a pose message by adding a vertex (& odometry edge) to the graph optimizer
        
        Arguments:
            posemsg {PoseStamped} -- A newly recorded pose in the system
        """
        self.vertex_seq += 1
        self.posegraph.add_vertex(self.vertex_seq, posemsg.pose.position.x)
        rospy.loginfo("Added pose vertex {0} with estimated position {1}".format(self.vertex_seq, posemsg.pose.position.x))
        
        if self.prev_pose is not None:
            dx = self.prev_pose.position.x - posemsg.pose.position.x

            # Add an edge from the previous pose to the current one
            self.posegraph.add_edge(fromVertex: self.vertex_seq - 1, 
                                    toVertex: self.vertex_seq, 
                                    dx: dx,
                                    information=np.identity(6))
            rospy.loginfo("    => Added edge from previous pose to this one. dx:{0}".format(dx))

        # TODO: Add to the output file
        self.prev_pose = posemsg.pose

    def on_scan(self, scanmsg):
        """Handles a scan message by adding a landmark edge to the graph optimizer
        
        Arguments:
            scanmsg {LaserScan} -- A new scan reading
        """
        scanmsg = LaserScan()
        
        if self.prev_pose is None:
            # Can't provide an estimate to the wall position until we get our first pose measurement.
            return

        # Note: for the robot, the first element in the laser scan message faces forward
        forward_range = scanmsg.ranges[0]

        if self.wall_vertex is None:
            x_estimate = self.prev_pose.position.x + forward_range 
            self.posegraph.add_landmark(WALL_VERTEX_ID, x_estimate)
            rospy.loginfo("Created landmark node at estimated position {0}".format(x_estimate))
            
        self.posegraph.add_landmark_edge(self.vertex_seq, WALL_VERTEX_ID, forward_range)
        
        rospy.loginfo("Added landmark edge from vertex {0} to wall at distance {1}".format(self.vertex_seq, forward_range))

    def spin(self):
        rospy.spin()

if __name__ == "__main__":

    optimizer = Optimizer()
    optimizer.spin()