#!/usr/bin/env python
"""
    optimizer.py

    Author: Julien Blanchet
    Feb. 8 2020

    A localization package that uses graph optimization (with the g20py library)
    to estimate position
"""

import rospy
import g2o


# vertex = g2o.VertexPointXY()