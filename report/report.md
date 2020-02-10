# Programming Assignment 3
Julien Blanchet - CS169 - 2/9/2020

**KEY LINKS**
* Bag file (using Mingi's file) https://drive.google.com/open?id=1onVc88q--nnUFm7Kv0Schqn1i-yoMVre
* Github repository with code https://github.com/j55blanchet/cs169_julienb_pa3
<hr>

## Description

### Iterations
* First bug: `dx` was negative
* Second bug: the post-optimization graph was exactly the same as the pre-optimization graph. I thought this could have something to do with the parameters (initially, I left the information matrix as the identity, which indicates a high uncertainty with regards to the sensor input).

### Choosing Parameters
In an effort to choose reasonable parameters for the covariance / information matrix of the graph optimization, I did some research online as to the accuracy of the ROSBot's sensors. I found that the LIDAR distance resolution [was listed as](https://www.robotshop.com/en/rplidar-a2m8-360-laser-scanner.html) `0.5mm (minimum)1.0% @ < 1.5 m (all distance range)`. This phrasing was somewhat unclear, but I took `1.0%` to be referring to the accuracy and thus computed the x value of the information matrix as `1 / 0.01 * range`. To determine a covariance parameter for the wheel odometry, I referred to [Husarion's documentation on the wheel encoders](https://husarion.com/manuals/rosbot-manual/#hardware-guide), which specified 48 pulses per revolution, corresponding to a 7.5 deg detection angle. Based on a 85mm wheel diameter, this computes to a pulse for every 5.56mm of forward / backwards motion. Over 1m of travel, we can thus expect the accuracy to be +/- 2.28mm, which I take as the standard deviation (67% confidence range). Thus, for the wheel odometry, I get a information value of `1 / 0.00228`, or `438`.



## Evaluation 


## Allocation of Effort
All programming work was done by myself. I used the answers posted in Piazza to help my work, and I used Mingi's bag file as the data source.
