/*
 * TrjCost.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#include "TrjCost.h"
#include <cmath>
#include "Constants.h"

using namespace std;
TrjCost::TrjCost() {
	// TODO Auto-generated constructor stub

}

/*
   Penalizes trajectories that span a duration which is longer or
   shorter than the duration requested.
 */
double TrjCost::time_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	double t = traj.t;
	double unperturbed_t = traj.unperturbed_t;

	return logistic(float(abs(t-unperturbed_t)) / unperturbed_t);
}
/*
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

Useful for cost functions.
 */
double TrjCost::logistic(double x){

	return 2.0 / (1 + exp(-x)) - 1.0;

}

/*
   Penalizes trajectories whose s coordinate (and derivatives)
    differ from the goal.
 */
double TrjCost::s_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	const vector<double> &s_coeff = traj.s_coeff;
	const vector<double> &unperturbed_s = traj.unperturbed_s;
	double t = traj.t;

	const vector<double> s = s_coeff;
	const vector<double> s_dot = differentiate(s);
	const vector<double> s_dot_dot = differentiate(s_dot);

	vector<double> s_actual = {to_equation(s, t),to_equation(s_dot, t),to_equation(s_dot_dot, t)};

	double cost = 0;
	for (int i =0; i< unperturbed_s.size(); i++){
		double actual = s_actual[i];
		double expected = unperturbed_s[i];
		double sigma = SIGMA_S[i];

		double diff = float(abs(actual-expected));
		cost += logistic(diff/sigma);
	}
	return cost;
}
/*
 Calculates the derivative of a polynomial and returns the corresponding coefficients.
 */
std::vector<double> TrjCost::differentiate(const std::vector<double> &coefficients){
	std::vector<double> new_cos = {};
	for (int i=0; i<coefficients.size();i++ ){
		if (i==0){
			continue;
		}
		new_cos.push_back(i * coefficients[i]);
	}
	return new_cos;
}
/*
 Takes the coefficients of a polynomial and point t, calculate the f value
 */
double TrjCost::to_equation(const std::vector<double> &coefficients, double t){
	double total = 0.0;
	for (int i = 0; i<coefficients.size(); i++ ){
		total += coefficients[i] * pow(t, i);
	}
	return total;
}

TrjCost::~TrjCost() {
	// TODO Auto-generated destructor stub
}

/*
   Penalizes trajectories whose d coordinate (and derivatives)
    differ from the goal.
 */
double TrjCost::d_diff_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	const vector<double> &d_coeff = traj.d_coeff;
	const vector<double> &unperturbed_d = traj.unperturbed_d;
	double t = traj.t;

	const vector<double> d = d_coeff;
	const vector<double> d_dot = differentiate(d);
	const vector<double> d_dot_dot = differentiate(d_dot);

	vector<double> d_actual = {to_equation(d, t),to_equation(d_dot, t),to_equation(d_dot_dot, t)};

	double cost = 0;
	for (int i =0; i< unperturbed_d.size(); i++){
		double actual = d_actual[i];
		double expected = unperturbed_d[i];
		double sigma = SIGMA_D[i];

		double diff = float(abs(actual-expected));
		cost += logistic(diff/sigma);
	}
	return cost;
}

double TrjCost::nearest_approach(const TrjObject &traj, const Vehicle &vehicle){
	double closest = 999999;
	const vector<double> &s_coeffs = traj.s_coeff;
	const vector<double> &d_coeffs = traj.d_coeff;
	double t = traj.t;
	for(int i=0; i< 100; i++){
		double cur_t = float(i) / 100 * t;
		double cur_s = to_equation(s_coeffs, cur_t);
		double cur_d = to_equation(d_coeffs, cur_d);
		std::vector<double> v_state = vehicle.state_in(cur_t);

		double targ_s = v_state[0];
		double targ_d = v_state[3];
		double dist = sqrt(pow((cur_s-targ_s), 2) + pow((cur_d-targ_d), 2));
		if(dist < closest){
			closest = dist;
		}
	}
	return closest;

}
/*
  Calculates the closest distance to any vehicle during a trajectory.
 */
double TrjCost::nearest_approach_to_any_vehicle(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	double closest = 999999;
	for (const auto& kv : predictions) {
		double d = nearest_approach(traj, kv.second);
		if(d < closest){
			closest = d;
		}
	}
	return closest;

}
/*
 *  Binary cost function which penalizes collisions.
 */
double TrjCost::collision_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	double nearest = nearest_approach_to_any_vehicle(traj, predictions);
	if(nearest < 2*VEHICLE_RADIUS)
		return 1.0;
	else
		return 0.0;
}
/*
 * Penalizes getting close to other vehicles.
 */

double TrjCost::buffer_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	double nearest = nearest_approach_to_any_vehicle(traj, predictions);
	return logistic(2*VEHICLE_RADIUS / nearest);
}

