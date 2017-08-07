# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Overview
In this project, our goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic.  
we are provided with the car's localization, sensor fusion data, as well as highway map data that lists the waypoints around the highway, by he simulator on a reqular freqeuncy(every 20 milliseconds), path palnner echos back the the next trajectory the simulator should follow.

## Final Result  

[Here](https://youtu.be/VhpZXRPkfrs) is the video that demonstrates the vehicle guided by our path planner successfully drives around the track in the simulator.

![mpc](./mpc.png)

## Valid Trajectories Checklist  

###The car is able to drive at least 4.32 miles without incident..
In the video above, the cars drives 5.59 miles without any accident.

###The car drives according to the speed limit.
The car does not drive beyond speed limit and always try driving around speed limit if possible. This is acheived by the trajectory layer. See Implementation section for more details.

###Max Acceleration and Jerk are not Exceeded.
During the driving, max acceleration and jerk are not exceeded. This is again achieved by the trajectory layer. See Implementation section for more details.

###Car does not have collisions.

No collisions occur in the video. This is achieved by collaboration from predictions, behavior and trajectory layer. See Implementation section for more details.

###The car stays in its lane, except for the time between changing lanes.
The car always stays in the center of the lane, except during lane chaning. This is achieved by the behavior layer. Its state changing FSM ensure the car does not move outside of valid lanes.

###The car is able to change lanes
The car would change lanes when it's appropriate. The decision making is guided by the behavior layer.See Implementation section for more details.


## Implementations

In general, A path planner consists of three layers: predictions, behavior and trajectory. Below we will illustrate how we use these three layers to generate the path.

###  Prediction
Prediction is about predicting future trajectories for other traffic on the road, based on sensor fusion data. This is important as we don’t want to bump into other cars while driving and thus have to incorporate traffic information when planning the path.

Three common approaches used for prediction are model based, data driven approach, and hybrid approach. Model based approach predefine a set of process models that the vehicle is likely to follow, iterate the models to generate corresponding trajectories, and finally use Multiple Model Algorithm to classify driving intent and thus get the probability of each likely trajectory. Data driven approach use traffic trajectory data and machine learning to directly predict trajectories. Hybrid approach is similar to model based, except that it uses machine learning and traffic data to classify driving intent.

In our project, we choose a very simple process model (constant velocity model) for all vehicles to simplify things. This obviously has its limitation, but it’s a reasonable assumption as the other vehicles in highway do not really change lanes very often. 

### Behavior

Behavior layer suggests driving strategy and pass it to trajectory layer, based on car localization and output from prediction layer.

This part is implemented in Behavior.cpp and BehavCost.cpp files.

In this project, the car could be in three states: Keep lane, lane change left, lane change right. We have to consider a lot of factors when choosing the state, like will we go faster after choosing a state, will it collide with other cars, is it comfortable for passengers?  Here we use a series of cost function to balance these sometimes conflicting requirements and select the state with minimal cost. To be specific, blow cost functions are applied.

1.	Lane speed cost
This is to prevent the car from going to lanes with leading car whose speed is very slow.
2.	Lane collision cost
This is to prevent the car from making moves that might collide with other cars
3.	Lane change cost
This is to prevent the car from changing the lanes too frequently.
4.	Lane change resoluteness cost
This one is very interesting. It is to reward the car to have resolute lane change. That is to say, once the car starts a lane change move, it’s better that it should stick with decision and complete the lane change. Otherwise sometimes the car might move like a drunk man, start moving to the left lane, and then half way through, decides to abort the decision and revert back.  

```
double lane_change_resoluteness_cost(const Vehicle & vehicle, BehvStates state, BehvCostData &data){
	if(data.last_state == BehvStates::LCL || data.last_state == BehvStates::LCR){
		int target_lane_id = get_target_laneid(vehicle, state);
		if(target_lane_id != data.last_intended_laneid){
			return 1;
		}
	}
	return 0;
}
```

### Trajectory

Trajectory layer generates optimal waypoints for the car to follow, based on sensor fusion data and the output of behavior layer and prediction layer.

This part is implemented in TrjMgr.cpp, Trajectory.cpp, and TrjCost.cpp.

Jerk Minimizing Trajectoris(JMT) is used to generate most comfortable trajectory from starting point (s_start, s_dot_start, s_dot_dot_start, d_start, d_dot_start, d_dot_dot_start) to ending point (s_end, s_dot_end, s_dot_dot_end, d_end, d_dot_end, d_dot_dot_end.).

As we’ve already known the starting point via localization data, the problem now is to find the appropriate ending points and send it to JMT. Behavior layer does provide some hints on the ending point, like which lane the ending point should be in, but trajectory has to figure out a lot of details on its own, like what s_end to use.

In this project, Trajectory layer specifies a certain number of candidate ending points and then use cost functions to select the best ending point (correspondingly the best trajectory).

For example, say we decide to follow a vehicle right before us in the same lane, we are 60 meters behind the leading car, and we deem 70 meters distance to be an ideal safe distance. Normally we would want our car to 70 meters behind the leading car in our next path planning. But if the leading car comes to a full stop, 70 meters would be a bad idea, as it basically asks our car to drive backward, which is barely feasible and more dangerous than just hitting the leading car. On the other side, if the leading car is driving very fast and our car currently is driving very slowly, we might exceed speed limit if we still go for 70 meters gap. In this case, the solution is to try from 5 to 100 gap distance meters and see which one is best, kind of brute force computation and but is a very effective method to handle various complicated scenarios that might occur. 

In this project, the cost functions used to select best trajectory are implemented in TrjCost.cpp, and includes:
* s_diff_cost
Penalizes trajectories whose s coordinate (and derivatives) differ from the goal.
* collision_cost
* exceeds_speed_limit_cost
* total_accel_cost
* max_accel_cost
* total_jerk_cost
* max_jerk_cost



## Reflections

This is definitely a very interesting and challenging project, a few take away from this project.  
* When we need to design an optimal value, and the choice of this optimal value depends on a wide variety of complicated scenarios. In this case, it’s barely feasible to implement the decision making with straightforward if else approach. One solution would be to try out all possible values in a brute force matter and see which one is best if we know beforehand that the optimal value should fall in a certain range and we can design proper cost functions to evaluate the result.
* Currently there are some glitches in the Frenet coordinate to Cartesian coordinate conversion. A more refined version of map data might help in this regard.
* As we mentioned earlier, our prediction layers assumes a very simple constant velocity motion model for other traffic on the road. This assumption may not always hold true and cause us to misjudge the trajectory of other traffic and cause collision. 
* Current implementations can also be further improved by optimizing the cost function design for behavior and trajectory layer, to increase the robustness of the path planner.


   

