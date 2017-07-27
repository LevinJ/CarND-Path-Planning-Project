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

