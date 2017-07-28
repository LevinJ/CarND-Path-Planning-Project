/*
 * Trajectory.cpp
 *
 *  Created on: Jul 24, 2017
 *      Author: levin
 */

#include "Trajectory.h"

#include "Eigen/Dense"
#include "TrjCost.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


Trajectory::Trajectory() {
	m_cost_map["time_diff_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["s_diff_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["d_diff_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["max_jerk_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["total_jerk_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["collision_cost"] = CostFuncWeight(&time_diff_cost, 10);
	m_cost_map["buffer_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["max_accel_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["total_accel_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["exceeds_speed_limit_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["stays_on_road_cost"] = CostFuncWeight(&time_diff_cost, 1);


}

Trajectory::~Trajectory() {
	// TODO Auto-generated destructor stub
}

/*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
vector<double> Trajectory::JMT(vector< double> start, vector <double> end, double T)
{


    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

    return result;

}

/*
 Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).

    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/
     this trajectory.
 */

TrjObject Trajectory::PTG(const std::vector<double> &start_s, const std::vector<double> &start_d,
		const std::vector<TrjGoal> &all_goals, double T,const std::map<int, Vehicle> &predictions){
	vector<TrjObject> trajectories = {};

	for(const auto &goal: all_goals){

		vector<double> s_coefficients = JMT(start_s, goal.s_goal, goal.t);
		vector<double> d_coefficients = JMT(start_d, goal.d_goal, goal.t);
		TrjObject trjobj;
		trjobj.s_coeff = s_coefficients;
		trjobj.d_coeff = d_coefficients;
		trjobj.t = goal.t;
		trjobj.unperturbed_s = goal.unperturbed_s;
		trjobj.unperturbed_d = goal.unperturbed_d;
		trjobj.unperturbed_t = T;

		trajectories.push_back(trjobj);
	}
	//Find the trajectory with the lowest cost
	double min_cost =  INFINITY;
	TrjObject best;
	for(auto &trj: trajectories ){
		double cost = calculate_cost(trj, predictions);
		if(cost < min_cost){
			best =trj;
		}
	}
	return best;
}

double Trajectory::calculate_cost(const TrjObject &trajectory,  const std::map<int, Vehicle> &predictions,
			bool verbose){
	double cost = 0;
	for (auto& kv : m_cost_map){
		auto cost_func_pair = kv.second;
		double new_cost = cost_func_pair.weight * cost_func_pair.cost_func(trajectory, predictions);
		cost += new_cost;
		if(verbose){
			cout<< "cost for "<<kv.first << " is "<< new_cost << endl;
		}
	}
	return cost;
}
