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
#

class BehvCostData{
public:
	BehvCostData(const Vehicle &vehicle_p, std::map<int, Vehicle> &predictions_p,
			std::map<int, Vehicle> leading_vehicles_p): vehicle(vehicle_p), predictions(predictions_p){
		leading_vehicles = leading_vehicles_p;
	}
	std::map<int, Vehicle> &predictions;
	const Vehicle &vehicle;
	std::map<int, Vehicle> leading_vehicles;
};

double lane_speed_cost(const Vehicle & vehicle, std::string state, BehvCostData &data);
double lane_collision_cost(const Vehicle & vehicle,  std::string state, BehvCostData &data);

#endif /* SRC_BEHVCOST_H_ */
