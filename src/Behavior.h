/*
 * Behavior.h
 *
 *  Created on: Aug 2, 2017
 *      Author: levin
 */

#ifndef SRC_BEHAVIOR_H_
#define SRC_BEHAVIOR_H_

#include <string>
#include <map>
#include <vector>
#include "BehvCost.h"
#include "Helper.h"

typedef  double (*BehvCostFunPtr)(const Vehicle & vehicle, std::string state, BehvCostData &data);

class BehvCostFuncWeight{
public:
	BehvCostFuncWeight(){

	}
	BehvCostFuncWeight(BehvCostFunPtr cost_func_p, double weight_p){
		cost_func = cost_func_p;
		weight = weight_p;
	}
	BehvCostFunPtr cost_func;
	double weight;
};


class Behavior {
public:
	Behavior();
	virtual ~Behavior();
	std::string update_state(const std::vector<double> &start_s, const std::vector<double> &start_d,
			std::map<int, Vehicle> &predictions);
private:
	double calculate_cost(const Vehicle & vehicle, std::string state, BehvCostData &data);
	std::map<std::string, BehvCostFuncWeight> m_cost_map;
	BehvCostData compute_behv_cost_data(const Vehicle & vehicle, std::map<int, Vehicle> &predictions);
};

#endif /* SRC_BEHAVIOR_H_ */
