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
1. Download the bag file from https://drive.google.com/open?id=1onVc88q--nnUFm7Kv0Schqn1i-yoMVre and save it as `<workspaceroot>/src/julienb_graphoptimization/bags/mingi-record.bag`
1. Run `catkin_make` at the root of your workspace.

## Running
1. `roslaunch julienb_graphoptimization optimize.launch`