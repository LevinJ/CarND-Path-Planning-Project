/*
 * TrjCost.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#include "TrjCost.h"
#include <cmath>
#include "Constants.h"
#include <algorithm>

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

double TrjCost::stays_on_road_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	const vector<double> &d_coeffs = traj.d_coeff;
	double t = traj.t;
	vector<double> all_ds = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_ds.push_back(to_equation(d_coeffs, cur_t));
	}

	auto max_d = std::max_element(std::begin(all_ds), std::end(all_ds));
	auto min_d = std::min_element(std::begin(all_ds), std::end(all_ds));

	if(*max_d <= MAX_D && *min_d >=MIN_D){
		return 0;
	}
	return 1;
}

double TrjCost::exceeds_speed_limit_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){

	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);
	double t = traj.t;
	vector<double> all_vs = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_vs.push_back(to_equation(s_dot_coeffs, cur_t));
	}

	auto max_v = std::max_element(std::begin(all_vs), std::end(all_vs));
	auto min_v = std::min_element(std::begin(all_vs), std::end(all_vs));

	if(*max_v < SPEED_LIMIT && *min_v > MIN_SPEED){
		return 0;
	}
	return 1;
}


double TrjCost::total_accel_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
	double t = traj.t;

	double dt = t / 100.0;

	double cur_t= 0;
	double total_acc = 0;

	for(int i=0; i< 100; i++){
		cur_t = dt * i;
		double acc = to_equation(s_dot_dot_coeffs, cur_t);
		total_acc += abs(acc*dt);
	}

	double acc_per_second = total_acc / cur_t;

	return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC );

}


double TrjCost::max_accel_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
	double t = traj.t;

	vector<double> all_as = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_as.push_back(to_equation(s_dot_dot_coeffs, cur_t));
	}

	auto max_acc = std::max_element(std::begin(all_as), std::end(all_as));
	if (abs(*max_acc) > MAX_ACCEL){
		return 1;
	}else{
		return 0;
	}
}

double TrjCost::total_jerk_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);

	vector<double> s_dot_dot_dot_coeffs = differentiate(s_dot_dot_coeffs);
	double t = traj.t;

	double dt = t / 100.0;

	double cur_t= 0;
	double total_jerks = 0;

	for(int i=0; i< 100; i++){
		cur_t = dt * i;
		double jerk = to_equation(s_dot_dot_coeffs, cur_t);
		total_jerks += abs(jerk*dt);
	}

	double jerk_per_second = total_jerks / cur_t;

	return logistic(jerk_per_second / EXPECTED_ACC_IN_ONE_SEC );

}

double TrjCost::max_jerk_cost(const TrjObject &traj, const std::map<int, Vehicle> &predictions){
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);

	vector<double> s_dot_dot_dot_coeffs = differentiate(s_dot_dot_coeffs);
	double t = traj.t;

	vector<double> all_jerks = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_jerks.push_back(to_equation(s_dot_dot_dot_coeffs, cur_t));
	}

	auto max_jerk = std::max_element(std::begin(all_jerks), std::end(all_jerks));
	if (abs(*max_jerk) > MAX_JERK){
		return 1;
	}else{
		return 0;
	}
}

