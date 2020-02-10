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

WALL_VERTEX_ID = 1
RATE_HZ = 10
WAIT_TIME_BEFORE_OPTIMIZE = rospy.Duration(secs=5)

class PA3Optimizer:

    def __init__(self):
        rospy.init_node("graphoptimizer")

        self.posegraph = PoseGraph()
        self.rate = rospy.Rate(hz=RATE_HZ)

        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.on_pose, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)

        self.vertex_seq = WALL_VERTEX_ID # landmark has id 1. The pose vertices will start with id of 2
        self.prev_pose = None
        self.has_wall_vertex = False
        self.last_msg_time = rospy.Time.now()
    
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

            # Add an edge from the previous pose to the current one)
            self.posegraph.add_edge(fromVertex=self.vertex_seq - 1, 
                                    toVertex=self.vertex_seq, 
                                    dx=dx)
            rospy.loginfo("    => Added edge from previous pose to this one. dx:{0}".format(dx))

        # TODO: Add to the output file
        self.prev_pose = posemsg.pose
        self.last_msg_time = rospy.Time.now()

    def on_scan(self, scanmsg):
        """Handles a scan message by adding a landmark edge to the graph optimizer
        
        Arguments:
            scanmsg {LaserScan} -- A new scan reading
        """
        
        if self.prev_pose is None:
            # Can't provide an estimate to the wall position until we get our first pose measurement.
            return

        # Note: for the robot, the first element in the laser scan message faces forward
        forward_range = scanmsg.ranges[0]

        if not self.has_wall_vertex:
            x_estimate = self.prev_pose.position.x + forward_range 
            self.posegraph.add_landmark(0, x_estimate)
            self.has_wall_vertex = True
            rospy.loginfo("Created landmark node at estimated position {0}".format(x_estimate))
            
        self.posegraph.add_landmark_edge(self.vertex_seq, WALL_VERTEX_ID, forward_range)
        self.last_msg_time = rospy.Time.now()
        rospy.loginfo("Added landmark edge from vertex {0} to wall at distance {1}".format(self.vertex_seq, forward_range))

    def spin(self):

        while not rospy.is_shutdown() and   
              self.vertex_seq > WALL_VERTEX_ID and   # ensure we have at least 1 vertex (e.g. that the rosbag has started playing)
                rospy.Time.now() - self.last_msg_time < WAIT_TIME_BEFORE_OPTIMIZE:

            self.rate.sleep()

        rospy.loginfo("No new messages in {0}. Optimizing now".format(WAIT_TIME_BEFORE_OPTIMIZE))
        self.pose_sub.unregister()
        self.scan_sub.unregister()
        self.posegraph.optimize()
        self.print_results()

    def print_results(self):
        rospy.loginfo("Printing results...")


if __name__ == "__main__":

    optimizer = PA3Optimizer()
    optimizer.spin()