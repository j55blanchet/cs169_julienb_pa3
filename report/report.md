# Programming Assignment 3
Julien Blanchet - CS169 - 2/9/2020

**KEY LINKS**
* Bag file (using Mingi's file) https://drive.google.com/open?id=1onVc88q--nnUFm7Kv0Schqn1i-yoMVre
* Github repository with code https://github.com/j55blanchet/cs169_julienb_pa3
<hr>

## Description

### Design

Building off [a g2o python example](https://github.com/uoip/g2opy#pose-graph-optimization), I decided to separate my code into a graph-builder class that wraps the g2o library (`posegraph.py`) and a main file (`pa3optimizer.py`) that interacts with ROS to retrieve messages. These are both independent - `posegraph.py` doesn't depend on ROS and `pa3optimizer.py` doesn't depend on g2o.

For saving the `.g2o` files, I elected to use the builtin `.save()` method of the `g2o.SparseOptimizer` class. I didn't see a need to reinvent the wheel here.

During tested, I also created an `optimize_from_file.py` as an alternate means to quickly test the solver on prewritten graph files. This ended up being quite useful, so I included it in the submission.

### Choosing Parameters
In an effort to choose reasonable parameters for the covariance / information matrix of the graph optimization, I did some research online as to the accuracy of the ROSBot's sensors. I found that the LIDAR distance resolution [was listed as](https://www.robotshop.com/en/rplidar-a2m8-360-laser-scanner.html) `0.5mm (minimum) 1.0% @ < 1.5 m (all distance range)`. This phrasing was somewhat unclear, but I took `1.0%` to be referring to the accuracy and thus computed the x value of the information matrix as `1 / 0.01 * range`.  __In practice, this led to values in the 60-70 range__. 

To determine a covariance parameter for the wheel odometry, I referred to [Husarion's documentation on the wheel encoders](https://husarion.com/manuals/rosbot-manual/#hardware-guide), which specified 48 pulses per revolution, corresponding to a 7.5 deg detection angle. Based on a 85mm wheel diameter, this computes to a pulse for every 5.56mm of forward / backwards motion. Over 1m of travel, we can thus expect the accuracy to be +/- 2.28mm, which I take as the standard deviation (67% confidence range). Thus, for the wheel odometry, I get an information value of `1 / 0.00228`, or `438`.

This was sufficient for the purposes of this assignment, but I do wonder which informaiton matrix value *should* be higher. If we "trust" the LIDAR sensor more, then it should probably have a higher information matrix value than the wheel odometry measurement.

### Factorization Strategy
While implementing factorization of landmark edges, I noticed that it wouldn't make sense to form a dense graph in which all robot pose nodes were connected to all other robot pose nodes, as the resulting problem size could grow quite quickly with E_landmark = θ(V!) where V is the number of nodes and E is number of landmark edges, no different than simply leaving the landmark node in the graph). I observed that visual odometry / loop closure constraints are more valuable between robot positions that are far apart, where more drift could have occured.

Based off this observation, I decided on a factorization scheme that added visual-odometry edge constraints between every `n` nodes, configurable by the `LANDMARK_FACTORIZATION_QUOTIENT` constant (could easily be a rospy parameter if needed). This way, the number of landmark edges is only (V/n)!. This accounts for drift but substanially reduces problem size, and future versions could omit more edges.

## Evaluation 

### Optimizer Performance

The optimizer I made seems to be functional, although I'm sure improvements could be made by adjusting the information matrix parameters. Examining table 1 below (also plotted in figure 1), you can see that the graph optimization did improve the distance estimate by about 6mm. From figure 2 you can see that the optimized graph follows the estimated pose graph quite closely. I attribute this to two factors: first, the information matrix parameters I put in are much higher for the odometry contraints than the visual constraints, meaning the scan measurements have less corrective influence on the outcome. Second, this was quite a short trial and there wasn't that much drift. In a longer, more difficult trial I would expect the optimized trajectory to show a much more distinct improvement over the pose estimates acquired from raw wheel odometry.

|  | Raw | Optimized | Ground Truth |
| --- | --- | ---       |      --- |
| Distance Traveled | 1.05936 | 1.06605 | 1.1 |
| Start | 2.70562 | 2.69591 | 0.9 |
| End | 3.76498 | 3.76196 | 2 |
*Table 1: Estimation error - graph optimization with factorized landmark constraints*

`<INSERT VISUALIZATION HERE>`


`<INSERT SECOND VISUALIZATION HERE>`


### Investigating Divergance

The biggest issue I encountered while testing my optimizer was that the post-optimization graph was exactly the same as the pre-optimization graph. I thought this could have something to do with the parameters (initially, I left the information matrix as the identity, which indicates a high uncertainty with regards to the sensor input). By running my `optimize_from_file.py` script (which outputs verbose debugging info), I was able to generate output which indicated that the solution was diverging (see the appendix for the exact output). See appendix for the exact output. I created a toy example to test my graph file structure and ended up discovering that my graph file was corrupt, in that landmark edges were referencing a landmark node that didn't exist, ending up with -1 as one of the vertex ids. I think the library would do well to generate more robust error messages in cases like this.

### Experimenting with Toy Examples

During investigation of the causes of divergance, I created a toy example, which led me to some unrelated but interesting observations. For all of these experiments, I used variations of a base graph that had 3 robot poses whose ground truth locations were at x = 0, x = 1, and x = 3, as well as a "wall" (landmark) whose location was at x = 5. The actual estimated positions and edge values on the graph file were purposely slightly inaccurate in order to provide an opportunity for error minimization. See appendix for the toy problem graph file.

| Observation | Supporting Experiment |
| --- | --- |
| Edges with information values of zero don't affect anything | Optimizing graph with and without a zero-information matrix edge |
| Increasing the information value of an edge increases the edge's influence on the final optimization| Doubling the information values of landmark edges|
| Changing node types affects outcome in subtle and ultimately unimportant ways. (Same point of conversion but different number of iterations and floating point rounding artifacts) | Optimization before and after changing type of the landmark (`VERTEX_XY` to `VERTEX_SE2`) and corresponding edges (`EDGE_SE2_XY` to `EDGE_SE2`)|
| The values of the information matrices only matter in relation to one another. Specifically, the relative values affect the conversion points. (However, increasing the absolute values does increase number of iterations of the solver) | Putting 100 for all edges in the information matrices instead of 1|
| The relative "weight" imparted by infomation matrix values is not linear in nature. | case 1 having odom edge info values of 1 and landmark of 2 versus case 2 with odom edges of 2 and landmark of 4.|

## Allocation of Effort
All programming work was done by myself. I used the answers posted in Piazza to help my work, and I used Mingi's bag file as the data source.

## Appendix

### Divergence Investigation
Output was the following (note that lambda increases, indicating divergance).

    iteration= 0     chi2= 0.000000  time= 0.000606434       cumTime= 0.000606434    edges= 56       schur= 0        lambda= 0.005848        levenbergIter= 1
    iteration= 1     chi2= 0.000000  time= 0.000190145       cumTime= 0.000796579    edges= 56       schur= 0        lambda= 0.003899        levenbergIter= 1
    iteration= 2     chi2= 0.000000  time= 0.000170761       cumTime= 0.00096734     edges= 56       schur= 0        lambda= 0.002599        levenbergIter= 1
    iteration= 3     chi2= 0.000000  time= 0.000168627       cumTime= 0.00113597     edges= 56       schur= 0        lambda= 0.001733        levenbergIter= 1
    iteration= 4     chi2= 0.000000  time= 0.000699408       cumTime= 0.00183538     edges= 56       schur= 0        lambda= 3633.787570     levenbergIter= 6



### Toy Graph (base values)
    VERTEX_SE2 1    -0.0   0   0
    VERTEX_SE2 2    0.9    0   0 
    VERTEX_SE2 3    3.1    0   0  
    VERTEX_XY  4    5.0    0


    EDGE_SE2 2 3  2.5 0 0    1 0 0 1 0 1
    EDGE_SE2 1 2  1.5 0 0    1 0 0 1 0 1

    EDGE_SE2_XY 1 4  5 0      1 0 1
    EDGE_SE2_XY 2 4  4 0      1 0 1
    EDGE_SE2_XY 3 4  2 0      1 0 1
