/*
 * Helper.h
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#ifndef EXERCISE_TRAJECTORYCPLUSPLUS_SRC_HELPER_H_
#define EXERCISE_TRAJECTORYCPLUSPLUS_SRC_HELPER_H_

#include <vector>
#include <map>

class Vehicle{

	//Helper class. Non-ego vehicles move w/ constant acceleration
public:
	std::vector<double> start_state;

	Vehicle(std::vector<double> start){
		start_state = start;
	}


	std::vector<double> state_in(double t){
		std::vector<double> s = {start_state[0],start_state[1],start_state[2]};
		std::vector<double> d = {start_state[3],start_state[4],start_state[5]};
		std::vector<double> state = {
				s[0] + (s[1] * t) + s[2] * t* t / 2.0,
				s[1] + s[2] * t,
				s[2],
				d[0] + (d[1] * t) + d[2] * t* t / 2.0,
				d[1] + d[2] * t,
				d[2],
		};
		return state;
	}
};

class Helper {
public:
	Helper();
	std::map<int, Vehicle> parse_sensor_fusion(std::vector<std::vector<double>> sensor_fusion);
	virtual ~Helper();
};



#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_HELPER_H_ */
