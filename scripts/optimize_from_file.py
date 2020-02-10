#!/usr/bin/env python
"""
    optimize_from_file.py

    Copied from https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/simple_optimize/simple_optimize.cpp
    Adapted for 2D scenario by Julien Blanchet
    
    Feb. 8 2020

    Reads in a g2o file, performs optimization, and outputs the results
"""
from __future__ import print_function

import numpy as np
import g2o 
import os

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-n', '--max_iterations', type=int, default=10, help='perform n iterations')
parser.add_argument('-i', '--input', type=str, default='sphere2500.g2o', help='input file')
parser.add_argument('-o', '--output', type=str, default='', help='save resulting graph as file')
args = parser.parse_args()

def main():
    #solver = g2o.BlockSolverX(g2o.LinearSolverCholmodX())
    # solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)

    optimizer = g2o.SparseOptimizer()
    optimizer.set_verbose(True)
    optimizer.set_algorithm(solver)

    optimizer.load(args.input)
    print('num vertices:', len(optimizer.vertices()))
    print('num edges:', len(optimizer.edges()), end='\n\n')

    optimizer.initialize_optimization()
    optimizer.optimize(args.max_iterations)

    if len(args.output) > 0:
        optimizer.save(args.output)

if __name__ == '__main__':
    assert os.path.isfile(args.input), (
        'Please provide a existing .g2o file')
        
    main()