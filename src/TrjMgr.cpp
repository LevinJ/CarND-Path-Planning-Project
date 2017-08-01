/*
 * TrjMgr.cpp
 *
 *  Created on: Jul 30, 2017
 *      Author: levin
 */

#include "TrjMgr.h"
#include "cmath"
#include "TrjCost.h"
#include "Helper.h"
#include "Constants.h"
#include "spline.h"

using namespace std;

static tk::spline m_x_spline; //put it here to avoid the compilation warning from the spline libray
static tk::spline m_y_spline;
static tk::spline m_dx_spline;
static tk::spline m_dy_spline;

std::ostream& operator<< (std::ostream& out, const TrjObject& trj) {
	out<<"s_coeff "<<trj.s_coeff<<endl;
	out<<"d_coeff "<<trj.d_coeff<<endl;
	out<<"t "<<trj.t<<endl;
	out<<"unperturbed_s "<<trj.unperturbed_s<<endl;
	out<<"unperturbed_d "<<trj.unperturbed_d<<endl;
	out<<"unperturbed_t "<<trj.unperturbed_t<<endl;
	out<<"s_goal "<<trj.s_goal<<endl;
	out<<"d_goal "<<trj.d_goal<<endl;

	double t = trj.t;
	vector<double> s = trj.s_coeff;
	vector<double> s_dot = differentiate(s);
	vector<double> s_d_dot = differentiate(s_dot);


	vector<double> S = {to_equation(s, t), to_equation(s_dot,t), to_equation(s_d_dot,t)};
	out<<"S "<<S<<endl;

	vector<double> d = trj.d_coeff;
	vector<double> d_dot = differentiate(d);
	vector<double> d_d_dot = differentiate(d_dot);


	vector<double> D = {to_equation(d, t), to_equation(d_dot,t), to_equation(d_d_dot,t)};
	out<<"D "<<D<<endl;

	return out;
}

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

TrjMgr::TrjMgr(const vector<double> &maps_s, const vector<double> &maps_x, vector<double> &maps_y,
		const vector<double> &maps_dx, const vector<double> &maps_dy) {
	m_maps_s = maps_s;
	m_maps_x = maps_x;
	m_maps_y = maps_y;
	m_maps_dx = maps_dx;
	m_maps_dy = maps_dy;

	m_x_spline.set_points(m_maps_s, m_maps_x);
	m_y_spline.set_points(m_maps_s, m_maps_y);
	m_dx_spline.set_points(m_maps_s, m_maps_dx);
	m_dy_spline.set_points(m_maps_s, m_maps_dy);
	m_last_waypoints_num = 0;

}

TrjMgr::~TrjMgr() {
	// TODO Auto-generated destructor stub
}

void TrjMgr::generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
		const std::vector<std::vector<double>> &sensor_fusion){

	static int loop = 1;

	cout<<"###loop "<< loop++<<endl;

	std::vector<std::vector<double>> start_state = get_start_state(car_state, previous_path_x,
			previous_path_y, end_path_s, end_path_d);
	std::vector<double> &start_s = start_state[0];
	std::vector<double> &start_d = start_state[1];
	std::map<int, Vehicle> predictions = get_predictons(sensor_fusion);
	double T = 200;
	TrjObject trjobj = m_trajectory.keep_lane(start_s, start_d, T, predictions);
	convert_next_waypoints(trjobj);

}
std::map<int, Vehicle> TrjMgr::get_predictons(const std::vector<std::vector<double>> &sensor_fusion){
	std::map<int, Vehicle> predictions;
	//	for (const auto& v : sensor_fusion)
	//	{
	//		int v_id = v[0];
	//		double x = v[1];
	//		double y = v[2];
	//		double vx = v[3];
	//		double vy = v[4];
	//		double s = v[5];
	//		double d = v[6];
	//		double s_dot = sqrt(vx*vx + vy*vy);
	//		//assume the other proceed with constant velocity along the road
	//		std::vector<double> start_state = {s, s_dot, 0, d, 0, 0};
	//		predictions[v_id] = Vehicle(start_state);
	//	}
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

	//get starting point from car param
	vector<double> start_s = {car_s,car_speed,0};
	vector<double> start_d = {car_d,0,0};
	cout<<"car_s, car_d"<<start_s<<","<<start_d<<endl;

	if(previous_path_x.size() != 0){
		//get starting point from pre path
		std::vector<std::vector<double>> start_state = process_prevpath(previous_path_x, previous_path_y, end_path_s, end_path_d);
		start_s = start_state[0];
		start_d = start_state[1];
	}

	return {start_s,start_d};
}

std::vector<std::vector<double>> TrjMgr::process_prevpath(const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y, double end_path_s,double end_path_d){
	vector<double> start_s = {};
	vector<double> start_d = {};
	for(int i=0; i< previous_path_x.size(); i++){
		vector<double> sd = getFrenet_Q(previous_path_x[i], previous_path_y[i], 0);
		//		cout<<"prev s,d "<<sd[0]<<", "<<sd[1]<<endl;
	}

	//get starting point based on last stored path
	int consume_num =  m_last_waypoints_num - previous_path_x.size();
	cout<<"m_last_waypoints_num "<<m_last_waypoints_num<< "previous_path_x.size()"<<previous_path_x.size()<<endl;
	cout<<"consumed "<< consume_num <<endl;
	double cur_t = consume_num * FRAME_UPDATE_TIME;
	static bool firsttime = true;
	if(firsttime){
		cur_t = (consume_num + REUSE_PREV_POINTS_NUM)  * FRAME_UPDATE_TIME;
		firsttime = false;

	}else{
		cur_t = consume_num  * FRAME_UPDATE_TIME;
	}

	cout<<"cur_t: "<<cur_t<<endl;
	//clean up locally stored waypoints
	vector<double> temp_s;
	vector<double> temp_d;
	for(int i=0; i<m_last_waypoints_num; i++){
		if(i<consume_num){
			//already consumed by the simulator, skip them
			continue;
		}
		if(i>=consume_num + REUSE_PREV_POINTS_NUM){
			//we've already got REUSE_PREV_POINTS_NUM waypoints, exit
			break;
		}
		temp_s.push_back(m_last_waypoints_s[i]);
		temp_d.push_back(m_last_waypoints_d[i]);
	}
	m_last_waypoints_s = temp_s;
	m_last_waypoints_d = temp_d;


	vector<double> s_coeff = m_last_trjobj.s_coeff;
	vector<double> s_dot_coeff = differentiate(s_coeff);
	vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);
	start_s = {to_equation(s_coeff, cur_t),to_equation(s_dot_coeff, cur_t),to_equation(s_dot_dot_coeff, cur_t)};

	vector<double> d_coeff = m_last_trjobj.d_coeff;
	vector<double> d_dot_coeff = differentiate(d_coeff);
	vector<double> d_dot_dot_coeff = differentiate(d_dot_coeff);
	start_d = {to_equation(d_coeff, cur_t),to_equation(d_dot_coeff, cur_t),to_equation(d_dot_dot_coeff, cur_t)};

	cout<<"estimated_s, estimated_d"<<start_s<<","<<start_d<<endl;



	return {start_s,start_d};

}

void TrjMgr::convert_next_waypoints(const TrjObject &trjobj){
	cout<<endl<<"best trajectory:"<<endl<<trjobj<<endl;
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
		//push new waypoints on top of previous old waypoints
		m_last_waypoints_s.push_back(cur_s);
		m_last_waypoints_d.push_back(cur_d);

		cur_t += dt;
	}
	for(int i=0; i< m_last_waypoints_s.size(); i++){
		double cur_s = m_last_waypoints_s[i];
		double cur_d = m_last_waypoints_d[i];
//		vector<double> xy = getXY_Q(cur_s, cur_d);
		vector<double> xy = convert_sd_to_xy(cur_s, cur_d);
		m_next_x_vals.push_back(xy[0]);
		m_next_y_vals.push_back(xy[1]);
		cout<<"cur_s, cur_d "<<cur_s<<","<<cur_d<<endl;
	}
	m_last_waypoints_num = m_next_x_vals.size();
	m_last_trjobj = trjobj;
	//	cout<<"m_next_x_vals size, "<<m_next_x_vals.size()<<endl;

}

vector<double> TrjMgr::getXY_Q(double s, double d){
	return getXY(s, d, m_maps_s, m_maps_x, m_maps_y);
}

vector<double> TrjMgr::getFrenet_Q(double x, double y, double theta){
	return getFrenet( x,  y,  theta, m_maps_x, m_maps_y);
}

vector<double> TrjMgr::convert_sd_to_xy(const double s, const double d){
	const double x_edge = m_x_spline(s);
	const double y_edge = m_y_spline(s);
	const double dx = m_dx_spline(s);
	const double dy = m_dy_spline(s);

	const double x = x_edge + dx * d;
	const double y = y_edge + dy * d;

	return {x, y};
}
