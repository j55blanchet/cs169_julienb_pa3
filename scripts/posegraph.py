#!/usr/bin/env python
"""
    posegraph.py

    Author: Julien Blanchet
    Feb. 8 2020

    A utility the uses the g2opy library to optimize a pose & landmark graph
      * Forked from https://github.com/uoip/g2opy#pose-graph-optimization
"""

import g2o
import numpy as np

from geometry_msgs.msg import Pose

class PoseGraph():
    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        self.optimizer.set_algorithm(solver)

    def optimize(self, max_iterations=20):
        self.optimizer.initialize_optimization()
        self.optimizer.optimize(max_iterations)

    def add_vertex(self, id, x_estimate, fixed=False):
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        se2 = g2o.SE2(x_estimate, 0, 0)
        v_se2.set_estimate(se2)
        v_se2.set_fixed(fixed)
        
        self.optimizer.add_vertex(v_se2)

    def add_edge(self, fromVertex, toVertex, dx, information=np.identity(3)):

        edge = g2o.EdgeSE2()
        for i, v in enumerate([fromVertex, toVertex]):
            if isinstance(v, int):
                v = self.optimizer.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(g2o.SE2(dx, 0, 0))  # relative pose
        edge.set_information(information)
        self.optimizer.add_edge(edge)

    def add_landmark(self, id, x):
        landmark = g2o.VertexPointXY()
        landmark.set_id(id)
        landmark.set_estimate([x, 0])

        self.optimizer.add_vertex(landmark)

    def add_landmark_edge(self, vertex_id, landmark_id, dx, information=np.identity(2)):
        
        edge = g2o.EdgeSE2PointXY()

        for i, v in enumerate([vertex_id, landmark_id]):
            if isinstance(v, int):
                v = self.optimizer.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement([dx, 0])
        edge.set_information(information)

        self.optimizer.add_edge(edge)

    def __str__(self):
        return "posegraph ({0} v, {1} e)".format(len(self.optimizer.vertices()), len(self.optimizer.edges()))

    def get_pose(self, id):
        return self.optimizer.vertex(id).estimate()

    def save(self, filepath):
        self.optimizer.save(filepath)
