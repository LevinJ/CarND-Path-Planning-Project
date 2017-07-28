/*
 * Trajectory.h
 *
 *  Created on: Jul 24, 2017
 *      Author: levin
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <map>
#include <iostream>
#include "Helper.h"
using namespace std;

typedef  double (*CostFunPtr)(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
class CostFuncWeight{
public:
	CostFuncWeight(){

	}
	CostFuncWeight(CostFunPtr cost_func_p, double weight_p){
		cost_func = cost_func_p;
		weight = weight_p;
	}
	CostFunPtr cost_func;
	double weight;
};

class Trajectory {
public:
	Trajectory();
	virtual ~Trajectory();
	std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);
private:

	std::map<std::string, CostFuncWeight> m_cost_map;
	TrjObject PTG(const std::vector<double> &start_s, const std::vector<double> &start_d,
			const std::vector<TrjGoal> &all_goals, double T,const std::map<int, Vehicle> &predictions);
	double calculate_cost(const TrjObject &trajectory,  const std::map<int, Vehicle> &predictions,
			bool verbose=false);

};

#endif /* TRAJECTORY_H_ */
