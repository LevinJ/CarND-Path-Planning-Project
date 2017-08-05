/*
 * Helper.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#include "Helper.h"
#include <math.h>
#include "Constants.h"
#include "TrjCost.h"
#include <sys/time.h>


std::ostream& operator<< (std::ostream& out, const TrjObject& trj) {
	out<<"s_coeff "<<trj.s_coeff<<endl;
	out<<"d_coeff "<<trj.d_coeff<<endl;
	out<<"t "<<trj.t<<endl;
	out<<"unperturbed_s "<<trj.unperturbed_s<<endl;
	out<<"unperturbed_d "<<trj.unperturbed_d<<endl;
	out<<"unperturbed_t "<<trj.unperturbed_t<<endl;
	out<<"s_goal "<<trj.s_goal<<endl;
	out<<"d_goal "<<trj.d_goal<<endl;
//
//	double t = trj.t;
//	vector<double> s = trj.s_coeff;
//	vector<double> s_dot = differentiate(s);
//	vector<double> s_d_dot = differentiate(s_dot);
//
//
//	vector<double> S = {to_equation(s, t), to_equation(s_dot,t), to_equation(s_d_dot,t)};
//	out<<"S "<<S<<endl;
//
//	vector<double> d = trj.d_coeff;
//	vector<double> d_dot = differentiate(d);
//	vector<double> d_d_dot = differentiate(d_dot);
//
//
//	vector<double> D = {to_equation(d, t), to_equation(d_dot,t), to_equation(d_d_dot,t)};
//	out<<"D "<<D<<endl;

	return out;
}

Helper::Helper() {
	// TODO Auto-generated constructor stub

}

std::map<int, Vehicle> Helper::parse_sensor_fusion(std::vector<std::vector<double>> sensor_fusion){
	std::map<int, Vehicle> predictions;
	for (const auto& v : sensor_fusion)
	{
		double id = v[0];
		double x = v[1];
		double y = v[2];
		double vx = v[3];
		double vy = v[4];
		double s = v[5];
		double d = v[6];
		double s_dot = sqrt(vx*vx + vy*vy);
		std::vector<double> start_state = {s,s_dot,0,d,0,0};

		Vehicle veh(start_state);
		predictions[id] = veh;

	}
	return predictions;

}

Helper::~Helper() {
	// TODO Auto-generated destructor stub
}

int	get_lane_num(double d){
	return int(d / LANE_WIDTH);
}
double get_lane_dist(int lane_id){
	return lane_id*LANE_WIDTH + LANE_WIDTH/2;
}

/*
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

Useful for cost functions.
 */
double logistic(double x){

	return 2.0 / (1 + exp(-x)) - 1.0;

}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	timeval curTime;
	gettimeofday(&curTime, NULL);
	int milli = curTime.tv_usec / 1000;

	char buffer [80];
	strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

	char currentTime[84] = "";
	sprintf(currentTime, "%s:%d", buffer, milli);

    return currentTime;
}
