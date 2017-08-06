/*
 * BehvCost.h
 *
 *  Created on: Aug 2, 2017
 *      Author: levin
 */

#ifndef SRC_BEHVCOST_H_
#define SRC_BEHVCOST_H_

#include <map>
#include "Helper.h"

enum BehvStates {KL, LCL, LCR};

inline std::ostream& operator<< (std::ostream& out, BehvStates state) {
	const static vector<std::string> states_str = {"KL", "LCL", "LCR"};
	out << states_str[state];
	return out;
}

int get_target_laneid(const Vehicle & vehicle, BehvStates state);
class BehvCostData{
public:
	BehvCostData(const Vehicle &vehicle_p, std::map<int, Vehicle> &predictions_p,
			std::map<int, Vehicle> leading_vehicles_p,
			BehvStates last_state_p, double last_LC_elapsed_duration_p, int last_intended_laneid_p): vehicle(vehicle_p), predictions(predictions_p){
		leading_vehicles = leading_vehicles_p;
		last_state = last_state_p;
		last_LC_elapsed_duration = last_LC_elapsed_duration_p;
		last_intended_laneid = last_intended_laneid_p;
	}
	std::map<int, Vehicle> &predictions;
	const Vehicle &vehicle;
	std::map<int, Vehicle> leading_vehicles;
	BehvStates last_state;
	double last_LC_elapsed_duration;
	int last_intended_laneid;
};

double lane_speed_cost(const Vehicle & vehicle, BehvStates state, BehvCostData &data);
double lane_collision_cost(const Vehicle & vehicle,  BehvStates state, BehvCostData &data);
double lane_change_cost(const Vehicle & vehicle, BehvStates state, BehvCostData &data);
double lane_change_resoluteness_cost(const Vehicle & vehicle, BehvStates state, BehvCostData &data);

#endif /* SRC_BEHVCOST_H_ */
