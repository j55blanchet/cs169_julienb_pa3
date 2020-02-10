# PA3 - CS169
Julien Blanchet | Robotics Perception Systems | 2/9/20

<hr>

## Requirements
* Ubuntu 16.04 with ROS Kinetic Installed
* g2opy library installed

## Setup

This repository is intended to be a *single package* within a catkin repository. Therefore, you must use an exising catkin workspace or create a new one. Steps for setup:

1. `cd` to the `src` folder of your catkin workspace
1. copy the contents of this repositrory into a folder titled `julienb_graphoptimization` (this is the package name)
1. Download the bag file from https://drive.google.com/open?id=1onVc88q--nnUFm7Kv0Schqn1i-yoMVre and save it as `<workspaceroot>/src/julienb_graphoptimization/data/mingi-record.bag`
1. Run `catkin_make` at the root of your workspace.

## Running
1. `roslaunch julienb_graphoptimization optimize.launch`
    * This will create a g2o graph from the file in data/ (default is mingi-record.bag), run the optimizer, and save the pre- and post- optimization graphs in the data/ folder
    * You can choose whether the program should factorize landmark edges or not - to do so, edit the `factorize_landmark_edges` parameter in the launch file or specify it on the commmand line as a roslaunch argument.

2. If you want to optimize a graph offline, you can also go to the `scripts` folder and run `python optimize_from_file.py -i /path/to/inputgraph.g2o -o /path/for/outputfile.g2o -n <numberOfIterations>`