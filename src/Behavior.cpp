/*
 * Behavior.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: levin
 */

#include "Behavior.h"
#include <cmath>
#include<iostream>
#include <algorithm>


using namespace std;

Behavior::Behavior() {
	m_cost_map["lane_speed_cost"] = BehvCostFuncWeight(&lane_speed_cost, 1);
	m_cost_map["lane_collision_cost"] = BehvCostFuncWeight(&lane_collision_cost, 10);
	m_cost_map["lane_change_cost"] = BehvCostFuncWeight(&lane_change_cost, 10);
	m_cost_map["lane_change_resoluteness_cost"] = BehvCostFuncWeight(&lane_change_resoluteness_cost, 100);
	m_last_state = BehvStates::KL;
	m_last_intended_laneid = 1;
}

Behavior::~Behavior() {


}

vector<BehvStates> Behavior::get_all_states(const std::vector<double> &start_d){
	vector<BehvStates> states = {};
	int cur_lane_id = get_lane_num(start_d[0]);

	switch(m_last_state){
	case BehvStates::KL:
		states = {BehvStates::KL, BehvStates::LCL, BehvStates::LCR};
		break;
	case BehvStates::LCL:
		states = {BehvStates::KL, BehvStates::LCL};
		break;
	case BehvStates::LCR:
		states = {BehvStates::KL, BehvStates::LCR};
		break;
	}
	//remove impossible lane selection

	switch(cur_lane_id){
	case 0:
		//remove LCR
		states.erase(std::remove(states.begin(), states.end(), BehvStates::LCR), states.end());
		break;
	case 2:
		//remove LCL
		states.erase(std::remove(states.begin(), states.end(), BehvStates::LCL), states.end());
		break;
	case 1:
		break;
	default:
		cout<<"Error: unexpected lane num="<< cur_lane_id <<endl;
	}

	return states;

}

BehvStates Behavior::update_state(const std::vector<double> &start_s, const std::vector<double> &start_d,
		std::map<int, Vehicle> &predictions){
	static bool firsttime = true;
	if(firsttime){
		firsttime = false;
		m_clock.reset();
	}

	vector<BehvStates> states = get_all_states(start_d);

	Vehicle  vehicle({start_s[0], start_s[1], start_s[2], start_d[0], start_d[1], start_d[2]});

	BehvCostData data = compute_behv_cost_data(vehicle, predictions);
	double min_cost = INFINITY;
	BehvStates min_cost_state = BehvStates::KL;

	for(auto &state: states){
		double cur_cost = calculate_cost(vehicle, state, data);
		if(cur_cost < min_cost){
			min_cost = cur_cost;
			min_cost_state = state;
		}
	}
	cout<<"min_cost_state="<<min_cost_state<<endl;
	if(m_last_state != min_cost_state){
		m_last_state = min_cost_state;
		m_last_intended_laneid = get_target_laneid(vehicle, m_last_state);
		m_clock.reset();
		cout<<"lane change instruction: lane="<<get_lane_num(start_d[0])<<" "<< min_cost_state<<endl;
	}

	return min_cost_state;


}


double Behavior::calculate_cost(const Vehicle & vehicle, BehvStates state, BehvCostData &data){
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
			cout<<"Error: unexpected lane in sensor fusion data" <<endl;
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
	BehvCostData data(vehicle, predictions, leading_vehicles, m_last_state, m_clock.elapsed(), m_last_intended_laneid);
	return data;
}

