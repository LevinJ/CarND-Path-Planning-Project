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

extern bool g_debugtrj;
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
	//	m_cost_map["time_diff_cost"] = CostFuncWeight(&time_diff_cost, 1);
	m_cost_map["s_diff_cost"] = CostFuncWeight(&s_diff_cost, 8);
	//	m_cost_map["d_diff_cost"] = CostFuncWeight(&d_diff_cost, 1);
	m_cost_map["max_jerk_cost"] = CostFuncWeight(&max_jerk_cost, 100);
	m_cost_map["total_jerk_cost"] = CostFuncWeight(&total_jerk_cost, 1);
	m_cost_map["collision_cost"] = CostFuncWeight(&collision_cost, 100);
	m_cost_map["buffer_cost"] = CostFuncWeight(&buffer_cost, 1);
	m_cost_map["max_accel_cost"] = CostFuncWeight(&max_accel_cost, 100);
	m_cost_map["total_accel_cost"] = CostFuncWeight(&total_accel_cost, 1);
	m_cost_map["exceeds_speed_limit_cost"] = CostFuncWeight(&exceeds_speed_limit_cost, 100);
	m_cost_map["stays_on_road_cost"] = CostFuncWeight(&stays_on_road_cost, 100);
	m_cur_car_speed = 0;
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
		std::vector<TrjObject> &all_trjs, double T,const std::map<int, Vehicle> &predictions){

	double min_cost =  INFINITY;
	TrjObject best;
	int count = 0;
	int best_count = 0;
	for(auto &trj: all_trjs){


		trj.s_coeff = JMT(start_s, trj.s_goal, trj.t);
		trj.d_coeff = JMT(start_d, trj.d_goal, trj.t);
		if(g_debugtrj){
			cout<<"###trjid="<<count<<", intended gap="<<trj.intended_gap<<", intended duration="<<trj.t<<endl;
		}


		double cost = calculate_cost(trj, predictions, g_debugtrj);
		if(cost < min_cost){
			best =trj;
			min_cost = cost;
			best_count = count;
		}
		count++;
	}
	cout<<"best trjid=" << best_count<< ", min cost ="<<min_cost<<", intended gap="<<best.intended_gap<<"\nbest trj="<<best<<endl;
	if(min_cost >=100){
		best.baccident = true;
	}else{
		best.baccident = false;
	}
	if(!g_debugtrj){
		calculate_cost(best, predictions, true);
	}
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
	if(verbose){
		cout<<"overall cost="<< cost<<"\ntrj="<<trajectory<<endl;
	}
	return cost;
}

std::vector<TrjObject> Trajectory::perturb_goals(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
		std::vector<double> &goal_s, std::vector<double> &goal_d,
		int target_vehicle, const std::vector<double> &delta,std::map<int, Vehicle> &predictions){

	//if we are to follow vehicles, the target gap sometimes it's hard to determin
	//due to the various scenarios that might occur (for example, the leading vehicle comes to
	//a still stop, the leading vehicle is driving past speed limit)
	static bool firsttime = true;
	static vector<double> delta_s_distances={};
	if(firsttime){
		firsttime = false;
		double min_dist = COLLISION_DISTANCE;
		double max_dist = SAFE_DISTANCE_BUFFER + 30;
		double d_s_dist = 5;

		double s_distance = min_dist;
		while(s_distance < max_dist){
			delta_s_distances.push_back(-s_distance);
			s_distance += d_s_dist;
		}
	}


	std::vector<TrjObject> all_trjs = {};

	if(target_vehicle == -1){
		//reduce s a bit to avoid possible collision with leading cars
		double gap_s = 0;
		while(gap_s <= 50){
			vector<double> purturbed_goal_s = goal_s;
			purturbed_goal_s[0] = purturbed_goal_s[0] - gap_s;
			if(m_cur_car_speed >= 22.3){
				purturbed_goal_s[1] -= 2;
				cout<<"reached critical speed, slow down a bit"<<endl;
			}
			all_trjs.push_back(TrjObject(purturbed_goal_s,goal_d,T,goal_s,goal_d, T, gap_s));
			gap_s += 5;
		}

		return all_trjs;
	}


	const Vehicle &target = predictions[target_vehicle];
	vector<double> target_vehicle_state = target.state_in(T);

	for(const auto &delta_distace:delta_s_distances){



		vector<double> purturbed_goal_s = {target_vehicle_state[0] + delta_distace, target_vehicle_state[1]+delta[1],0};
		if(purturbed_goal_s[0] < 0){
			//s should always be positve number
			purturbed_goal_s[0] = 5;
		}
		if(purturbed_goal_s[1] > SPEED_LIMIT){
			//make sure we do not exceed speed limit when following vehicles
			purturbed_goal_s[1] = SPEED_LIMIT;
		}
		if(m_cur_car_speed >= 22.3){
			purturbed_goal_s[1] -= 2;
			cout<<"reached critical speed, slow down a bit"<<endl;
		}
		vector<double> purturbed_goal_d = {target_vehicle_state[3] + delta[3],
				target_vehicle_state[4]+delta[4],target_vehicle_state[5]+delta[5]};
		//the only we purturbed here is the s
		goal_s= {target_vehicle_state[0] - SAFE_DISTANCE_BUFFER, purturbed_goal_s[1], purturbed_goal_s[2]};
		goal_d = purturbed_goal_d;
		all_trjs.push_back(TrjObject(purturbed_goal_s,purturbed_goal_d,T,goal_s,goal_d, T, delta_distace));
	}
	return all_trjs;

}

TrjObject Trajectory::follow_goal(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
		std::vector<double> &goal_s, std::vector<double> &goal_d,  std::map<int, Vehicle> &predictions){
	std::vector<TrjObject> all_goals = perturb_goals(start_s, start_d, T, goal_s, goal_d, -1, {},predictions);
	return PTG(start_s, start_d,all_goals, T,predictions);

}

TrjObject Trajectory::follow_vehicle(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
		int target_vehicle, const std::vector<double> &delta,  std::map<int, Vehicle> &predictions){
	std::vector<double> goal_s = {};
	std::vector<double> goal_d = {};
	std::vector<TrjObject>  all_goals = perturb_goals(start_s, start_d, T, goal_s,
			goal_d, target_vehicle, delta,predictions);
	return PTG(start_s, start_d,all_goals, T,predictions);
}

TrjObject Trajectory::keep_lane(const std::vector<double> &start_s, const std::vector<double> &start_d,
		double T, std::map<int, Vehicle> &predictions){

	double s = start_s[0];
	double d = start_d[0];
	int leading_id = -1;
	double distance = INFINITY;
	//Find leading vehicle in the same lane
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
		//if we have leading vehicle, check whether it's within safe distance
		double cur_distance = predictions[leading_id].start_state[0] - start_s[0];
		if (cur_distance < SAFE_DISTANCE_BUFFER){
			//make sure we do not change lanes
			double delta_d = get_lane_dist(get_lane_num(start_d[0])) - predictions[leading_id].start_state[3];
			vector<double> delta = {0, 0,0,delta_d,0,0};
			cout<<"trj: keep lane, has target "<<leading_id<<", target vehicle state="<<predictions[leading_id].start_state<<endl;

			TrjObject trjobj =  follow_vehicle(start_s, start_d, T, leading_id, delta,  predictions);
			cout<<"old distance="<<cur_distance<<", new distance="<<trjobj.intended_gap <<endl;
			if(cur_distance <= 10){
				cout<<"hmm, kind of emergency here."<<endl;
			}
			return trjobj;
		}
	}

	//follow a reasonably set goal
	double planed_a = (SPEED_LIMIT - start_s[1])/T;

	cout<<"trj: keep lane, no target, "<<planed_a<<endl;
	vector<double> goal_s = {start_s[0]+ start_s[1]*T + 0.5*planed_a*T*T, SPEED_LIMIT, 0};


	vector<double> goal_d = {get_lane_dist(get_lane_num(start_d[0])),0,0};
	TrjObject trjobj =  follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions);
	cout<<"old distance="<<0<<", new distance="<< trjobj.intended_gap <<endl;
	return trjobj;
}

TrjObject Trajectory::LC(const std::vector<double> &start_s, const std::vector<double> &start_d,
		double T, std::map<int, Vehicle> &predictions, bool left){

	int target_lane_id = left ? get_lane_num(start_d[0]) - 1 :  get_lane_num(start_d[0]) + 1;
	string lc_action_str = left ? " LCL " : " LCR ";
	double distance = INFINITY;
	int leading_id = -1;
	for(auto &kv: predictions){
		//find lading vehicle in the target lane
		Vehicle &v = kv.second;
		if(get_lane_num(v.start_state[3]) != target_lane_id || v.start_state[0] < start_s[0]){
			continue;
		}
		if((v.start_state[0] -start_s[0])< distance){
			distance = v.start_state[0] -start_s[0];
			leading_id = kv.first;
		}
	}
	if(leading_id !=-1){
		//if we have leading vehicle, check whether it's within safe distance
		double cur_distance = predictions[leading_id].start_state[0] - start_s[0];
		if (cur_distance < SAFE_DISTANCE_BUFFER){
			//let's increase the gap in a stable/gradual fashion
			//make sure we change to the center of the other lane
			double target_lane_dist = predictions[leading_id].start_state[3];
			double delta_d = get_lane_dist(get_lane_num(target_lane_dist)) - target_lane_dist;
			vector<double> delta = {0, 0,0,delta_d,0,0};


			TrjObject trjobj = follow_vehicle(start_s, start_d, T, leading_id, delta,  predictions);
			string trjres = trjobj.baccident ? " failure" : " success";
			cout<<"trj: "<<lc_action_str<<", has target "<<leading_id<<" delta, "<<delta<<trjres<<endl;
			cout<<"old distance="<<cur_distance<<", new distance="<< trjobj.intended_gap <<endl;
			return trjobj;
		}
	}

	//follow a reasonably set goal, namely reach the speed limit by T seconds
	double planed_a = (SPEED_LIMIT - start_s[1])/T;
	vector<double> goal_s = {start_s[0]+ start_s[1]*T + 0.5*planed_a*T*T, SPEED_LIMIT, 0};
	vector<double> goal_d = {get_lane_dist(target_lane_id),0,0};

	TrjObject trjobj = follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions);
	string trjres = trjobj.baccident ? " failure" : " success";
	cout<<"trj: "<<lc_action_str<< "no target, "<<planed_a<<trjres<<endl;
	cout<<"old distance="<<0<<", new distance="<< trjobj.intended_gap <<endl;
	return trjobj;
}

