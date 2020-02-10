#!/usr/bin/env python
"""
    pa3optimizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that constucts a pose-graph from suscribed ros topics
    and then performs bundle optimization on the graph. Designed specifically for a 1D
    localization scenario and only considers a single landmark - the front-facing scan.
"""

import sys
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from posegraph import PoseGraph

# Interesting constants - feel free to try things!
WAIT_TIME_BEFORE_OPTIMIZE = rospy.Duration(secs=5)
WHEEL_ODOMETRY_INFORMATION = 1 / 0.00228 # Corresponding to a std deviation of 2.28 mm over 1m of travel
SCAN_STDDEV_PERCENTAGE = 0.01
LANDMARK_FACTORIZATION_QUOTIENT = 10 # 1 / n poses will be connected with landmark edges

# No reason to modify these constants
WALL_VERTEX_ID = 1
RATE_HZ = 10

class PA3Optimizer:

    def __init__(self):
        rospy.init_node("graphoptimizer")

        self.posegraph = PoseGraph()
        self.rate = rospy.Rate(hz=RATE_HZ)

        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.on_pose, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)

        self.factorize_landmark_edges = rospy.get_param("~factorize_landmark_edges", default=True)

        self.vertex_seq = WALL_VERTEX_ID # landmark has id 1. The pose vertices will start with id of 2
        self.prev_pose = None
        self.last_msg_time = rospy.Time.now()

        self.has_wall_vertex = False
        self.prev_wallconstraint_vertexid = None
        self.landmark_key_vertices = []
        self.landmark_vertex_measurements = []
    
    def on_pose(self, posemsg):
        """Handles a pose message by adding a vertex (& odometry edge) to the graph optimizer
        
        Arguments:
            posemsg {PoseStamped} -- A newly recorded pose in the system
        """
        self.vertex_seq += 1
        self.posegraph.add_vertex(self.vertex_seq, posemsg.pose.position.x)
        rospy.loginfo("POSE #{0: 2d} @ {1: .3f}".format(self.vertex_seq, posemsg.pose.position.x))
        
        if self.prev_pose is not None:
            dx = posemsg.pose.position.x - self.prev_pose.position.x

            info_matrix = np.identity(3)
            info_matrix[0][0] = WHEEL_ODOMETRY_INFORMATION

            # Add an edge from the previous pose to the current one)
            self.posegraph.add_edge(fromVertex=self.vertex_seq - 1, 
                                    toVertex=self.vertex_seq, 
                                    dx=dx,
                                    information=info_matrix)
            rospy.loginfo("    => dx: {0:0.3f}".format(dx))

        # TODO: Add to the output file
        self.prev_pose = posemsg.pose
        self.last_msg_time = rospy.Time.now()

    def on_scan(self, scanmsg):
        """Handles a scan message by adding a landmark edge to the graph optimizer
        
        Arguments:
            scanmsg {LaserScan} -- A new scan reading
        """
        
        if self.prev_pose is None:
            rospy.logwarn("Ignoring scan b/c of lack of prev pose")
            # Can't provide an estimate to the wall position until we get our first pose measurement.
            return

        if self.prev_wallconstraint_vertexid is self.vertex_seq:
            rospy.logwarn("Ignoring scan b/c we already encoded a constraint to the current vertex")
            return

        # Note: for the robot, the first element in the laser scan message faces forward
        x_dist = scanmsg.ranges[0]

        if np.isinf(x_dist):
            rospy.logwarn("Ignoring scan b/c of infinite measurement")
            return

        x_std_dev = x_dist * SCAN_STDDEV_PERCENTAGE
        x_information = 1.0 / x_std_dev
        x_pos_estimate = self.prev_pose.position.x + x_dist 
        
        if self.factorize_landmark_edges:
            self.addscan_factored(x_pos_estimate, x_information, x_dist)
        else:
            self.addscan_nonfactored(x_pos_estimate, x_information, x_dist)
        

    def addscan_nonfactored(self, x_pos_estimate, x_information, x_dist):
        
        if not self.has_wall_vertex:
            self.posegraph.add_landmark(WALL_VERTEX_ID, x_pos_estimate)
            self.has_wall_vertex = True
            rospy.loginfo("Created landmark node at estimated position {0: .4f}".format(x_pos_estimate))
            
        info_matrix = np.identity(2)
        info_matrix[0][0] = x_information

        self.posegraph.add_landmark_edge(   \
            vertex_id=self.vertex_seq,      \
            landmark_id=WALL_VERTEX_ID,     \
            dx=x_dist,                      \
            information=info_matrix)

        self.prev_wallconstraint_vertexid = self.vertex_seq

        self.last_msg_time = rospy.Time.now()
        rospy.loginfo("    {0} to WALL: range {1: .4f}   est pos:{2: .4f}".format(self.vertex_seq, x_dist, x_pos_estimate))


    def addscan_factored(self, x_pos_estimate, x_information, x_dist):

        if self.prev_wallconstraint_vertexid is not None and \
           self.vertex_seq - self.prev_wallconstraint_vertexid < LANDMARK_FACTORIZATION_QUOTIENT:
            # This isn't a key vertex (one that will have landmark constraints added to it)
            return
            
        # Add an edge between current vertex and all existing key vertices
        for v, v_x_dist in zip(self.landmark_key_vertices, self.landmark_vertex_measurements):
           
            dx = v_x_dist - x_dist

            info_matrix = np.identity(3)
            info_matrix[0][0] = x_information # TODO: this should perhaps be an average of information value of current and previous scan

            # Add an edge from the previous pose to the current one)
            self.posegraph.add_edge(fromVertex=v, 
                                    toVertex=self.vertex_seq, 
                                    dx=dx,
                                    information=info_matrix)

        rospy.loginfo("LANDMARK-FACTORIZED: #{0: 2d} landmark_x_dist:{1: .4f}".format(self.vertex_seq, x_dist))    
        self.landmark_vertex_measurements.append(x_dist)
        self.landmark_key_vertices.append(self.vertex_seq)
        self.prev_wallconstraint_vertexid = self.vertex_seq

    def spin(self):

        # Ensure we have at least 1 vertex (e.g. that the rosbag has started playing), 
        #   and then wait a bit after the last message before starting the optimization routine
        while not rospy.is_shutdown() and   \
              (self.vertex_seq == WALL_VERTEX_ID or rospy.Time.now() - self.last_msg_time < WAIT_TIME_BEFORE_OPTIMIZE):

            self.rate.sleep()

        rospy.loginfo("No new messages in {0}. Moving on".format(WAIT_TIME_BEFORE_OPTIMIZE))
        self.optimize()

    def optimize(self):
        self.pose_sub.unregister()
        self.scan_sub.unregister()

        preop_file = rospy.get_param("~preoptimization_output")
        rospy.loginfo("Graph construction complete. Printing graph to {0}".format(preop_file))
        self.posegraph.save(preop_file)

        self.posegraph.optimize()

        postop_file = rospy.get_param("~postoptimization_output")
        rospy.loginfo("Optimization complete. Printing graph to {0}".format(postop_file))
        self.posegraph.save(postop_file)

if __name__ == "__main__":

    optimizer = PA3Optimizer()
    optimizer.spin()