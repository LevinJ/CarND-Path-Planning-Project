/*
 * Behavior.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: levin
 */

#include "Behavior.h"
#include <cmath>
#include<iostream>

using namespace std;

Behavior::Behavior() {

	m_cost_map["lane_speed_cost"] = BehvCostFuncWeight(&lane_speed_cost, 1);
	m_cost_map["lane_collision_cost"] = BehvCostFuncWeight(&lane_collision_cost, 10);
	m_cost_map["lane_change_cost"] = BehvCostFuncWeight(&lane_change_cost, 10);

}

Behavior::~Behavior() {
	m_last_state = "KL";

}

std::string Behavior::update_state(const std::vector<double> &start_s, const std::vector<double> &start_d,
		std::map<int, Vehicle> &predictions){
	return "KL";
	vector<string> states = {"KL", "LCL", "LCR"};
	Vehicle  vehicle({start_s[0], start_s[1], start_s[2], start_d[0], start_d[1], start_d[2]});
	int cur_lane_id = get_lane_num(vehicle.start_state[3]);
	if(cur_lane_id == 0){
		//remove LCR
		states.erase(states.begin() + 2);
	}

	if(cur_lane_id == 2){
		//remove LCL
		states.erase(states.begin() + 1);
	}
	BehvCostData data = compute_behv_cost_data(vehicle, predictions);
	double min_cost = INFINITY;
	string min_cost_state = "KL";

	for(auto &state: states){
		double cur_cost = calculate_cost(vehicle, state, data);
		if(cur_cost < min_cost){
			min_cost = cur_cost;
			min_cost_state = state;
		}
	}
	cout<<"min_cost_state="<<min_cost_state<<endl;
	if(min_cost_state == "LCL" || min_cost_state == "LCR"){
		m_clock.reset();
	}
	m_last_state = min_cost_state;
	return min_cost_state;


}


double Behavior::calculate_cost(const Vehicle & vehicle, std::string state, BehvCostData &data){
	double cost = 0;
	for (auto& kv : m_cost_map){
		auto cost_func_pair = kv.second;
		double cur_cost = cost_func_pair.weight * cost_func_pair.cost_func(vehicle, state, data);
		cost += cur_cost;

		cout<< "state="<<state<<" cost for "<<kv.first << " is "<< cur_cost << endl;
	}
	cout<< "state="<<state<<" cost="<<cost<< endl;
	return cost;
}

BehvCostData Behavior::compute_behv_cost_data(const Vehicle & vehicle, std::map<int, Vehicle> &predictions){
	std::map<int, Vehicle> leading_vehicles;
	double cur_s = vehicle.start_state[0];

	for (const auto& kv : predictions) {
		const Vehicle &v = kv.second;
		if(v.start_state[0] <= cur_s){
			//we only check vehicles ahead of us
			continue;
		}
		int lane_id = get_lane_num(v.start_state[3]);
		if(lane_id <0 || lane_id >2){
			//ignore such vehicles
			cout<<"error: unexpected lane in sensor fusion data" <<endl;
			continue;
		}
		if(leading_vehicles.find(lane_id) == leading_vehicles.end()){
			//no vehilce in this lane are in the map yet
			leading_vehicles[lane_id] = v;
			continue;
		}
		if(v.start_state[0] < leading_vehicles[lane_id].start_state[0]){
			leading_vehicles[lane_id] = v;
		}
	}

	BehvCostData data(vehicle, predictions, leading_vehicles);
	return data;
}

