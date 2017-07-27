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
	double logistic(double x);
	double time_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	double s_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions);
	std::vector<double> differentiate(const std::vector<double> &coefficients);
	double to_equation(const std::vector<double> &coefficients, double t);
};

#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_TRJCOST_H_ */
