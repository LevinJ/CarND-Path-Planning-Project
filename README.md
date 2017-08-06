# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Overview
In this project, our goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic.  
we are provided with the car's localization, sensor fusion data, as well as highway map data that lists the waypoints around the highway,  by he simulator on a reqular freqeuncy(every 20 milliseconds), path palnner echos back the the next trajectory the simulator should follow.

## Final Result  

[Here](https://youtu.be/VhpZXRPkfrs) is the video that demonstrates the vehicle guided by our path planner successfully drives around the track in the simulator.

![mpc](./mpc.png)

## Valid Trajectories Checklist  

###The car is able to drive at least 4.32 miles without incident..
In the video above, the cars drives 5.59 miles without any accident.

###The car drives according to the speed limit.
The cars does not drive beyond speed limit and alwys try drining around speed limit it possible. This is acheived by the trajectory layer. See Reflection section for more details.

###Max Acceleration and Jerk are not Exceeded.
During the driving, max acceleration and jerk are not exceeded. This is again achieved by the trajectory layer. See Reflection section for more details.

###Car does not have collisions.

No collisions occur in the video. This is achieved by collaboration from predictions, behavior and trajectory layer.See Reflection section for more details.

###The car stays in its lane, except for the time between changing lanes.
The car always stays in the center of the lane, except during lane chaning.

###The car is able to change lanes
The car would change lanes when it's appropriate. The decision making is guided by the behavior layer.See Reflection section for more details.


## Reflections

   

