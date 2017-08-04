/*
 * Helper.h
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#ifndef EXERCISE_TRAJECTORYCPLUSPLUS_SRC_HELPER_H_
#define EXERCISE_TRAJECTORYCPLUSPLUS_SRC_HELPER_H_

#include <vector>
#include <map>
#include <iterator>
#include <chrono>


using namespace std;


class ElapsedClock
{
public:
	ElapsedClock() : beg_(clock_::now()) {}
	void reset() { beg_ = clock_::now(); }
	double elapsed() const {
		return std::chrono::duration_cast<chrono::milliseconds>
		(clock_::now() - beg_).count(); }

private:
	typedef std::chrono::high_resolution_clock clock_;

	std::chrono::time_point<clock_> beg_;
};

class Vehicle{

	//Helper class. Non-ego vehicles move w/ constant acceleration
public:
	std::vector<double> start_state;

	Vehicle(const std::vector<double> &start){
		start_state = start;
	}
	Vehicle(){

	}

	//Here we assume the vehicle is in a constant acceleration, and zero turn rate motion model
	std::vector<double> state_in(double t) const{
		std::vector<double> s = {start_state[0],start_state[1],start_state[2]};
		std::vector<double> d = {start_state[3],start_state[4],start_state[5]};
		std::vector<double> state = {
				s[0] + (s[1] * t) + s[2] * t* t / 2.0,
				s[1] + s[2] * t,
				s[2],
				d[0] + (d[1] * t) + d[2] * t* t / 2.0,
				d[1] + d[2] * t,
				d[2],
		};
		return state;
	}
};


class TrjObject{
public:
	TrjObject(const std::vector<double> &s_goal_p, const std::vector<double> &d_goal_p,double t_p,
			const std::vector<double> &unperturbed_s_p,const std::vector<double> &unperturbed_d_p, double unperturbed_t_p){
		s_goal = s_goal_p;
		d_goal = d_goal_p;
		t = t_p;
		unperturbed_s = unperturbed_s_p;
		unperturbed_d = unperturbed_d_p;
		unperturbed_t = unperturbed_t_p;
		baccident = false;

	}
	TrjObject(){

	}
	std::vector<double> s_goal;
	std::vector<double> d_goal;
	std::vector<double> s_coeff;
	std::vector<double> d_coeff;
	double t;
	std::vector<double> unperturbed_s;
	std::vector<double> unperturbed_d;
	double unperturbed_t;
	bool baccident;
};

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
	if ( !v.empty() ) {
		out << '[';
		std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
		out << "]";
	}
	return out;
}


int	get_lane_num(double d);
double get_lane_dist(int lane_id);
double logistic(double x);

class Helper {
public:
	Helper();
	std::map<int, Vehicle> parse_sensor_fusion(std::vector<std::vector<double>> sensor_fusion);
	virtual ~Helper();
};

std::ostream& operator<< (std::ostream& out, const TrjObject& trj);

#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_HELPER_H_ */
