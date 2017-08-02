/*
 * Helper.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#include "Helper.h"
#include <math.h>
#include "Constants.h"

Helper::Helper() {
	// TODO Auto-generated constructor stub

}

std::map<int, Vehicle> Helper::parse_sensor_fusion(std::vector<std::vector<double>> sensor_fusion){
	std::map<int, Vehicle> predictions;
	for (const auto& v : sensor_fusion)
	{
		double id = v[0];
		double x = v[1];
		double y = v[2];
		double vx = v[3];
		double vy = v[4];
		double s = v[5];
		double d = v[6];
		double s_dot = sqrt(vx*vx + vy*vy);
		std::vector<double> start_state = {s,s_dot,0,d,0,0};

		Vehicle veh(start_state);
		predictions[id] = veh;

	}
	return predictions;

}

Helper::~Helper() {
	// TODO Auto-generated destructor stub
}

int	get_lane_num(double d){
	return int(d / LANE_WIDTH);
}
double get_lane_dist(int lane_id){
	return lane_id*LANE_WIDTH + LANE_WIDTH/2;
}

/*
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

Useful for cost functions.
 */
double logistic(double x){

	return 2.0 / (1 + exp(-x)) - 1.0;

}
