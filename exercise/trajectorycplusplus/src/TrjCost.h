/*
 * TrjCost.h
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#ifndef EXERCISE_TRAJECTORYCPLUSPLUS_SRC_TRJCOST_H_
#define EXERCISE_TRAJECTORYCPLUSPLUS_SRC_TRJCOST_H_

#include "vector"
#include "Helper.h"

class TrjCost {
public:
	TrjCost();
	virtual ~TrjCost();

	double time_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double s_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);

	double d_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);

	double collision_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double buffer_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double stays_on_road_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double exceeds_speed_limit_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double total_accel_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double max_accel_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double total_jerk_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double max_jerk_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
private:
	double nearest_approach(const TrjObject &traj, const Vehicle &vehicle);
	double nearest_approach_to_any_vehicle(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	std::vector<double> differentiate(const std::vector<double> &coefficients);
	double to_equation(const std::vector<double> &coefficients, double t);
	double logistic(double x);
};

#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_TRJCOST_H_ */
