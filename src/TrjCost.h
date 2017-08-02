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


	double time_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double s_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);

	double d_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);

	double collision_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double buffer_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double stays_on_road_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double exceeds_speed_limit_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double total_accel_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double max_accel_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double total_jerk_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);
	double max_jerk_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions, bool verbose=false);

	double nearest_approach(const TrjObject &traj, const Vehicle &vehicle);
	double nearest_approach_to_any_vehicle(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	std::vector<double> differentiate(const std::vector<double> &coefficients);
	double to_equation(const std::vector<double> &coefficients, double t);



#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_TRJCOST_H_ */
