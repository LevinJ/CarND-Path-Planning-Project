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

typedef  double (*CostFunPtr)(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose);
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
	TrjObject keep_lane(const std::vector<double> &start_s, const std::vector<double> &start_d,
			double T, std::map<int, Vehicle> &predictions);
	std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);
	TrjObject LC(const std::vector<double> &start_s, const std::vector<double> &start_d,
			double T, std::map<int, Vehicle> &predictions, bool left= true);
	double calculate_cost(const TrjObject &trajectory,  const std::map<int, Vehicle> &predictions,
				bool verbose=false);

private:

	std::map<std::string, CostFuncWeight> m_cost_map;
	TrjObject PTG(const std::vector<double> &start_s, const std::vector<double> &start_d,
			std::vector<TrjObject> &all_trjs, double T,const std::map<int, Vehicle> &predictions);

	std::vector<TrjObject> perturb_goals(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
			std::vector<double> &goal_s, std::vector<double> &goal_d,
			int target_vehicle, const std::vector<double> &delta, std::map<int, Vehicle> &predictions);
//	std::vector<std::vector<double>> perturb_goal(const std::vector<double> &goal_s, const std::vector<double> &goal_d);
	TrjObject follow_goal(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
			std::vector<double> &goal_s, std::vector<double> &goal_d,  std::map<int, Vehicle> &predictions);
	TrjObject follow_vehicle(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
			int target_vehicle, const std::vector<double> &delta,  std::map<int, Vehicle> &predictions);
};

#endif /* TRAJECTORY_H_ */
