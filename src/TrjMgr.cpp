/*
 * TrjMgr.cpp
 *
 *  Created on: Jul 30, 2017
 *      Author: levin
 */

#include "TrjMgr.h"
#include "cmath"
#include "TrjCost.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

TrjMgr::TrjMgr() {
	// TODO Auto-generated constructor stub

}

TrjMgr::~TrjMgr() {
	// TODO Auto-generated destructor stub
}

void TrjMgr::generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
		const std::vector<std::vector<double>> &sensor_fusion){

	std::vector<std::vector<double>> start_state = get_start_state(car_state, previous_path_x,
			previous_path_y, end_path_s, end_path_d);
	std::vector<double> &start_s = start_state[0];
	std::vector<double> &start_d = start_state[1];
	std::map<int, Vehicle> predictions = get_predictons(sensor_fusion);
	double T = 5;
	TrjObject trjobj = m_trajectory.keep_lane(start_s, start_d, T, predictions);
	convert_next_waypoints(trjobj);

}
std::map<int, Vehicle> TrjMgr::get_predictons(const std::vector<std::vector<double>> &sensor_fusion){
	std::map<int, Vehicle> predictions;
		for (const auto& v : sensor_fusion)
		{
			int v_id = v[0];
			double x = v[1];
			double y = v[2];
			double vx = v[3];
			double vy = v[4];
			double s = v[5];
			double d = v[6];
			double s_dot = sqrt(vx*vx + vy*vy);
			//assume the other proceed with constant velocity along the road
			std::vector<double> start_state = {s, s_dot, 0, d, 0, 0};
			predictions[v_id] = Vehicle(start_state);
		}
		return predictions;
}
std::vector<std::vector<double>> TrjMgr::get_start_state(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d){
	double car_x = car_state[0];
	double car_y = car_state[1];
	double car_s = car_state[2];
	double car_d = car_state[3];
	double car_yaw = car_state[4];
	double car_speed = car_state[5];
	cout<<"previous_path_x"<<previous_path_x<<endl;

	return {{car_s,car_speed,0},{car_d,0,0}};
}

void TrjMgr::convert_next_waypoints(const TrjObject &trjobj){
	m_next_x_vals = {};
	m_next_y_vals = {};
	double t = trjobj.t;
	std::vector<double> s_coeff = trjobj.s_coeff;
	std::vector<double> d_coeff = trjobj.d_coeff;
	double dt = 0.02;
	double cur_t = 0;
	while(cur_t <= t){
		double cur_s = to_equation(s_coeff, cur_t);
		double cur_d = to_equation(d_coeff, cur_t);
		vector<double> xy = getXY_Q(cur_s, cur_d);
		m_next_x_vals.push_back(xy[0]);
		m_next_y_vals.push_back(xy[1]);
		cur_t += dt;
	}

}

vector<double> TrjMgr::getXY_Q(double s, double d){
	return getXY(s, d, m_maps_s, m_maps_x, m_maps_y);
}

