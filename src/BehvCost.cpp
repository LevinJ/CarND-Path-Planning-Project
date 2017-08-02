/*
 * BehvCost.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: levin
 */

#include "BehvCost.h"
#include "Helper.h"
#include "Constants.h"
#include <cmath>

static int get_target_laneid(const Vehicle & vehicle, std::string state){
	int cur_lane_id = get_lane_num(vehicle.start_state[3]);
	int target_lane_id = cur_lane_id;
	if(state == "LCL"){
		target_lane_id = cur_lane_id + 1;
	}else if (state == "LCR"){
		target_lane_id = cur_lane_id -1;
	}
	return target_lane_id;

}

double lane_speed_cost(const Vehicle & vehicle, std::string state, BehvCostData &data){

	int target_lane_id = get_target_laneid(vehicle, state);
	double target_lane_speed = target_lane_speed = SPEED_LIMIT + 10;

	if(data.leading_vehicles.find(target_lane_id) != data.leading_vehicles.end()){
		target_lane_speed = data.leading_vehicles[target_lane_id].start_state[1];
	}

	return logistic(SPEED_LIMIT/target_lane_speed);
}

double lane_collision_cost(const Vehicle & vehicle, std::string state, BehvCostData &data){
	if(state == "KL"){
		return 0;
	}
	int target_lane_id = get_target_laneid(vehicle, state);
	double front_gap = INFINITY;
	double back_gap = INFINITY;

	if(data.leading_vehicles.find(target_lane_id) != data.leading_vehicles.end()){
		front_gap = data.leading_vehicles[target_lane_id].start_state[0] - vehicle.start_state[0];
	}

	//check the back cars
	for (const auto& kv : data.predictions) {
		const Vehicle &v = kv.second;
		if(get_lane_num(v.start_state[3]) != target_lane_id){
			continue;
		}
		if(v.start_state[0] > vehicle.start_state[0]){
			continue;
		}
		double gap = vehicle.start_state[0] - v.start_state[0];
		if(gap < back_gap){
			back_gap = gap;
		}
	}

	if(front_gap > FRONT_GAP_THRESH && back_gap> BACK_GAP_THRESH){
		return 0;
	}
	return 1;
}

