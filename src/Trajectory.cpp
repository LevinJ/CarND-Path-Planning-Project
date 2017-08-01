/*
 * Trajectory.cpp
 *
 *  Created on: Jul 24, 2017
 *      Author: levin
 */

#include "Trajectory.h"
#include <algorithm>
#include <functional>
#include <random>
#include <algorithm>

#include "Eigen/Dense"
#include "TrjCost.h"
#include "Constants.h"
#include "Helper.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
	assert(a.size() == b.size());

	std::vector<T> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
			std::back_inserter(result), std::plus<T>());
	return result;
}


Trajectory::Trajectory() {
	m_cost_map["time_diff_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["s_diff_cost"] = CostFuncWeight(&s_diff_cost, 1);
	m_cost_map["d_diff_cost"] = CostFuncWeight(&d_diff_cost, 1);
	m_cost_map["max_jerk_cost"] = CostFuncWeight(&max_jerk_cost, 1);
	m_cost_map["total_jerk_cost"] = CostFuncWeight(&total_jerk_cost, 1);
	m_cost_map["collision_cost"] = CostFuncWeight(&collision_cost, 10);
	m_cost_map["buffer_cost"] = CostFuncWeight(&buffer_cost, 1);
	m_cost_map["max_accel_cost"] = CostFuncWeight(&max_accel_cost, 1);
	m_cost_map["total_accel_cost"] = CostFuncWeight(&total_accel_cost, 1);
	m_cost_map["exceeds_speed_limit_cost"] = CostFuncWeight(&exceeds_speed_limit_cost, 1);
	m_cost_map["stays_on_road_cost"] = CostFuncWeight(&stays_on_road_cost, 1);


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
		trjobj.s_goal = goal.s_goal;
		trjobj.d_goal = goal.d_goal;

		trajectories.push_back(trjobj);
	}
	//Find the trajectory with the lowest cost
	double min_cost =  INFINITY;
	TrjObject best;
	for(auto &trj: trajectories ){
		double cost = calculate_cost(trj, predictions);
//		cout<<"s_goal "<<trj.s_goal<<"d_goal "<<trj.d_goal << "t "<<trj.t <<"cost "<<cost<<endl;
		if(cost < min_cost){
			best =trj;
			min_cost = cost;
		}
	}
	calculate_cost(best, predictions, true);
	return best;
}

double Trajectory::calculate_cost(const TrjObject &trajectory,  const std::map<int, Vehicle> &predictions,
		bool verbose){
	double cost = 0;
	for (auto& kv : m_cost_map){
		auto cost_func_pair = kv.second;
		double new_cost = cost_func_pair.weight * cost_func_pair.cost_func(trajectory, predictions, verbose);
		cost += new_cost;
		if(verbose){
			cout<< "cost for "<<kv.first << " is "<< new_cost << endl;
		}
	}
	return cost;
}

std::vector<TrjGoal> Trajectory::perturb_goals(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
		std::vector<double> &goal_s, std::vector<double> &goal_d,
		int target_vehicle, const std::vector<double> &delta,std::map<int, Vehicle> &predictions){
	std::vector<TrjGoal> all_goals = {};
	double timestep = 0.5;
	double t = T - 4 * timestep;
	bool folllow_vehicle = true;

	if(target_vehicle == -1){
		folllow_vehicle = false;
	}

	while(t <= T + 4 * timestep){
		if(folllow_vehicle){
			const Vehicle &target = predictions[target_vehicle];
			vector<double> target_state = target.state_in(t) + delta;
			goal_s = {target_state[0],target_state[1],target_state[2]};
			goal_d = {target_state[3],target_state[4],target_state[5]};
		}

		all_goals.push_back(TrjGoal(goal_s,goal_d,t,goal_s,goal_d));
//		cout<<"t "<< t <<", goal "<<goal_s<<","<< goal_d<<", unperturbed "<<goal_s<<","<< goal_d<<endl;
		for(int i=0; i<N_SAMPLES;i++){
			std::vector<std::vector<double>> perturbed = perturb_goal(goal_s, goal_d);

			all_goals.push_back(TrjGoal(perturbed[0], perturbed[1], t,goal_s,goal_d));
		}

		t += timestep;
	}
	return all_goals;
}

/*
 * Returns a "perturbed" version of the goal.
 */
std::vector<std::vector<double>> Trajectory::perturb_goal(const std::vector<double> &goal_s, const std::vector<double> &goal_d){
	vector<double> new_s_goal = {};
	//random goal s
	for(int i=0; i< goal_s.size(); i++){
		double x = goal_s[i];
		double sigma_s = SIGMA_S[i];
		random_device rd;
		default_random_engine gen(rd());
		normal_distribution<double> dist_x(x, sigma_s);
		new_s_goal.push_back(dist_x(gen));
	}
	//let's not change the acceleration
	new_s_goal[2] = goal_s[2];
	//random goal d
	//	vector<double> new_s_goal = {};
	//	for(int i=0; i< goal_d.size(); i++){
	//		double x = goal_d[i];
	//		double sigma_d = SIGMA_D[i];
	//		random_device rd;
	//		default_random_engine gen(rd());
	//		normal_distribution<double> dist_x(x, sigma_d);
	//		new_d_goal.push_back(dist_x(gen));
	//	}
	vector<double> new_d_goal = goal_d;
	return {new_s_goal, new_d_goal};
}

TrjObject Trajectory::follow_goal(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
		std::vector<double> &goal_s, std::vector<double> &goal_d,  std::map<int, Vehicle> &predictions){
	std::vector<TrjGoal> all_goals = perturb_goals(start_s, start_d, T, goal_s, goal_d, -1, {},predictions);
	return PTG(start_s, start_d,all_goals, T,predictions);

}

TrjObject Trajectory::follow_vehicle(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
		int target_vehicle, const std::vector<double> &delta,  std::map<int, Vehicle> &predictions){
	std::vector<double> goal_s = {};
	std::vector<double> goal_d = {};
	std::vector<TrjGoal>  all_goals = perturb_goals(start_s, start_d, T, goal_s,
			goal_d, target_vehicle, delta,predictions);
	return PTG(start_s, start_d,all_goals, T,predictions);
}
int	Trajectory::get_lane_num(double d){
	return int(d / LANE_WIDTH);
}
double Trajectory::get_lane_dist(int lane_id){
	return lane_id*LANE_WIDTH + LANE_WIDTH/2;
}
TrjObject Trajectory::keep_lane(const std::vector<double> &start_s, const std::vector<double> &start_d,
		double T, std::map<int, Vehicle> &predictions){
	bool has_target = false;

	double s = start_s[0];
	double d = start_d[0];
	int leading_id = -1;
	double distance = INFINITY;
	//Find leadig vehicle in the same lane
	for(auto &kv: predictions){
		Vehicle &v = kv.second;
		if(get_lane_num(v.start_state[3]) != get_lane_num(d) || v.start_state[0] < s){
			continue;
		}
		if((v.start_state[0] -s)< distance){
			distance = v.start_state[0] -s;
			leading_id = kv.first;
		}
	}
	if(leading_id !=-1){
		double max_distance = (SPEED_LIMIT - predictions[leading_id].start_state[1])* T;
		if (distance < max_distance + SAFE_DISTANCE_BUFFER){
			has_target = true;
		}
	}

	//follow the vehicle or follow the goal
	if(has_target){

		int target_vehicle = leading_id;
		//make we do not change lanes
		double delta_d = get_lane_dist(get_lane_num(start_d[0])) - predictions[target_vehicle].start_state[3];
		double deta_s = predictions[target_vehicle].start_state[0] - start_s[0];
		if(deta_s < SAFE_DISTANCE_BUFFER){
			//let's get away from the leading vehicle till a safe distance by and by
			deta_s = deta_s + 5;
		}else{
			deta_s = SAFE_DISTANCE_BUFFER;
		}
		vector<double> delta = {-deta_s, 0,0,0,0,0};
		cout<<"keep lane, has target "<<leading_id<<" delta, "<<delta<<endl;
		return follow_vehicle(start_s, start_d, T, target_vehicle, delta,  predictions);
	}else{
		double target_speed = (SPEED_LIMIT + start_s[1])/2;
		if (target_speed > SPEED_LIMIT){
			target_speed = SPEED_LIMIT;
		}
		cout<<"keep lane, no target, "<<target_speed<<endl;
		vector<double> goal_s = {s+ target_speed*T, SPEED_LIMIT, 0};


		vector<double> goal_d = {get_lane_dist(get_lane_num(start_d[0])),0,0};
		return follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions);
	}

}

TrjObject Trajectory::LC(const std::vector<double> &start_s, const std::vector<double> &start_d,
		double T, std::map<int, Vehicle> &predictions, bool prepare, bool left){
	bool has_target = false;
	double s = start_s[0];
	double d = start_d[0];
	int target_lane = -1;
	std::vector<double> delta = {};
	if(left)
		target_lane = get_lane_num(d) + 1;
	else
		target_lane = get_lane_num(d) - 1;
	if(prepare){
		if(left)
			delta = {0, 0,0,-LANE_WIDTH,0,0};
		else
			delta = {0, 0,0,LANE_WIDTH,0,0};

	}

	else{
		delta = {0, 0,0,0,0,0};
	}
	int closetest_id = -1;
	double distance = INFINITY;
	for(auto &kv: predictions){
		Vehicle &v = kv.second;
		if(get_lane_num(v.start_state[3]) != target_lane ){
			continue;
		}
		if((abs(v.start_state[0] -s))< distance){
			distance = abs(v.start_state[0] -s);
			closetest_id = kv.first;
		}
	}
	double max_distance = 0;
	double delta_s = 0;
	if(closetest_id !=-1){
		if(predictions[closetest_id].start_state[0] < s){
			max_distance = predictions[closetest_id].start_state[1] * T;
			delta_s = SAFE_DISTANCE_BUFFER;

		}else{
			max_distance = (SPEED_LIMIT - predictions[closetest_id].start_state[1])* T;
			delta_s = -SAFE_DISTANCE_BUFFER;

		}
		if(distance < max_distance){
			has_target = true;
		}

	}

	if(has_target){
		int target_vehicle = closetest_id;
		if(not prepare){
			delta[0] = delta_s;
		}
		return follow_vehicle(start_s, start_d, T, target_vehicle, delta,  predictions);

	}


	if(prepare){
		return keep_lane(start_s, start_d, T, predictions);
	}

	//lane change
	vector<double> goal_s = {s+ (SPEED_LIMIT + start_s[1])*T/2, SPEED_LIMIT, 0};
	vector<double> goal_d = {target_lane*LANE_WIDTH + LANE_WIDTH/2,0,0};
	return follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions);

}

