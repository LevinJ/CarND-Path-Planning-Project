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
	m_current_sate = BehvStates::KL;

}

TrjMgr::~TrjMgr() {
	// TODO Auto-generated destructor stub
}

void TrjMgr::generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
		const std::vector<std::vector<double>> &sensor_fusion){

	static int loop = 1;

	cout<<"###loop "<< loop++<<","<<currentDateTime()<<endl;
//	if(handle_lane_change(car_state, previous_path_x,previous_path_y,
//			end_path_s, end_path_d,sensor_fusion)){
//		return;
//	}

	std::vector<std::vector<double>> start_state = get_start_state(car_state, previous_path_x,
			previous_path_y, end_path_s, end_path_d);
	std::vector<double> &start_s = start_state[0];
	std::vector<double> &start_d = start_state[1];
	std::map<int, Vehicle> predictions = get_predictons(sensor_fusion, start_s[0], car_state);
	double T = 5;

	BehvStates suggested_state = m_behavior.update_state(start_s, start_d, predictions);
//	m_current_sate = suggested_state;

	TrjObject trjobj;
	if (suggested_state == BehvStates::LCL){
		trjobj = m_trajectory.LC(start_s, start_d,T, predictions, true);
	}
	if (suggested_state == BehvStates::LCR){
		trjobj = m_trajectory.LC(start_s, start_d,T, predictions, false);
	}
	if(suggested_state == BehvStates::KL){
//	if(suggested_state == BehvStates::KL || trjobj.baccident){
		trjobj = m_trajectory.keep_lane(start_s, start_d, T, predictions);
//		m_current_sate = BehvStates::KL;
	}
	convert_next_waypoints(trjobj);

}
std::map<int, Vehicle> TrjMgr::get_predictons(const std::vector<std::vector<double>> &sensor_fusion,
		double start_s,const std::vector<double> &car_state){
	std::map<int, Vehicle> predictions;
	int pred_count = 0;
	double closest_distance = INFINITY;
	double car_s = car_state[2];
	double car_d = car_state[3];
	double closetsid = -1;
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
		if(d<0){
			continue;
		}

		if(start_s <=MAX_S && start_s >= MAX_S - 300){
			//if JMT start point happens to be at the last segment of the lap
			//change traffic position accordingly so that leading vehicle can be properly detected
			if(s>=0){
				cout << "last segment of the lap, adjust vehicle position, vehilce id="<< v_id
						<<", s="<< s << " , new_s="<< s + MAX_S<<endl;
				s = s + MAX_S;
			}
		}


		//predict other traffic on the road into the future
		//since we are planning JMT for the future
		double new_s = s + s_dot * REUSE_PREV_POINTS_NUM * FRAME_UPDATE_TIME;

		//assume the other proceed with constant velocity along the road
		double cur_distance =  sqrt(pow((s-car_s), 2) + pow((d-car_d), 2));
		if(cur_distance < closest_distance){
			closest_distance = cur_distance;
			closetsid = pred_count;
		}
		std::vector<double> start_state = {new_s, s_dot, 0, d, 0, 0};
//		cout<<"vehicle s="<< s<<endl;
		cout<<"predictions["<<pred_count++<<"] = Vehicle("<<start_state<<");"<<endl;
		predictions[v_id] = Vehicle(start_state);
	}
	cout<<"closest traffic id="<<closetsid<<", distance="<<closest_distance<<endl;
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
	//from miles/hour to meter/s
	car_speed = (car_speed * 1609.34)/3600;

	m_trajectory.m_cur_car_speed = car_speed;

	//get starting point from car param
	vector<double> start_s = {car_s,car_speed,0};
	vector<double> start_d = {car_d,0,0};
	cout<<"car_s="<<start_s<<", car_d="<<start_d<<endl;

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

	//get starting point based on last stored path
	int consume_num =  m_last_waypoints_num - previous_path_x.size();

	//	cout<<"consumed="<< consume_num << ", m_last_waypoints_num="<<m_last_waypoints_num<< ", previous_path_x.size()="<<previous_path_x.size()<<endl;
	cout<<"consumed="<< consume_num <<endl;
	cout<<"estimated_s="<<m_last_waypoints_s[consume_num]<<", estimated_d="<<m_last_waypoints_d[consume_num]<<endl;



	start_s = m_last_waypoints_s[consume_num + REUSE_PREV_POINTS_NUM];
	start_d = m_last_waypoints_d[consume_num + REUSE_PREV_POINTS_NUM];;
	cout<<"vector<double> start_s = "<<start_s<<"; vector<double> start_d = "<<start_d<<";"<<endl;




	vector<vector<double>> temp_s;
	vector<vector<double>> temp_d;
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

	return {start_s,start_d};

}

void TrjMgr::convert_next_waypoints(const TrjObject &trjobj){
	//	cout<<endl<<"best trajectory:"<<endl<<trjobj<<endl;
	m_next_x_vals = {};
	m_next_y_vals = {};
	double t = trjobj.t;
	std::vector<double> s_coeff = trjobj.s_coeff;
	vector<double> s_dot_coeff = differentiate(s_coeff);
	vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);

	std::vector<double> d_coeff = trjobj.d_coeff;
	vector<double> d_dot_coeff = differentiate(d_coeff);
	vector<double> d_dot_dot_coeff = differentiate(d_dot_coeff);

	double dt = FRAME_UPDATE_TIME;
	double cur_t = 0;

	int count = All_POINTS_NUM - m_last_waypoints_s.size();
	for(int i=0; i< count; i++){
		if(cur_t > t){
			cout<<"All_POINTS_NUM being is a bit too big for this round"<<All_POINTS_NUM<<endl;
			break;
		}
		double cur_s = to_equation(s_coeff, cur_t);
		if(cur_s > MAX_S){
			cur_s = cur_s - MAX_S;
		}
		vector<double> s = {cur_s,to_equation(s_dot_coeff, cur_t),to_equation(s_dot_dot_coeff, cur_t)};
		m_last_waypoints_s.push_back(s);

		vector<double> d = {to_equation(d_coeff, cur_t),to_equation(d_dot_coeff, cur_t),to_equation(d_dot_dot_coeff, cur_t)};
		m_last_waypoints_d.push_back(d);

		cur_t += dt;
	}
	for(int i=0; i< m_last_waypoints_s.size(); i++){
		double cur_s = m_last_waypoints_s[i][0];
		double cur_d = m_last_waypoints_d[i][0];
		//		vector<double> xy = getXY_Q(cur_s, cur_d);
		vector<double> xy = convert_sd_to_xy(cur_s, cur_d);
		m_next_x_vals.push_back(xy[0]);
		m_next_y_vals.push_back(xy[1]);
		//		cout<<"cur_s, cur_d "<<cur_s<<","<<cur_d<<endl;
	}
	m_last_waypoints_num = m_next_x_vals.size();
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

bool TrjMgr::handle_lane_change(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
		const std::vector<std::vector<double>> &sensor_fusion){
	if(m_current_sate == BehvStates::KL){
		//always generate new trajectory when in kl state
		return false;
	}

	if(previous_path_x.size() < 50){
		//most of existing trajectory has been consumed, let's generate new trajectory
		return false;
	}
	int consume_num =  m_last_waypoints_num - previous_path_x.size();

	//remove consumed waypoints
	vector<vector<double>> temp_s;
	vector<vector<double>> temp_d;
	for(int i=0; i<m_last_waypoints_num; i++){
		if(i<consume_num){
			//already consumed by the simulator, skip them
			continue;
		}
		temp_s.push_back(m_last_waypoints_s[i]);
		temp_d.push_back(m_last_waypoints_d[i]);
	}


	m_last_waypoints_s = temp_s;
	m_last_waypoints_d = temp_d;
	m_last_waypoints_num = m_last_waypoints_s.size();
	m_next_x_vals = {};
	m_next_y_vals = {};
	for(int i=0; i< m_last_waypoints_s.size(); i++){
		double cur_s = m_last_waypoints_s[i][0];
		double cur_d = m_last_waypoints_d[i][0];
		//		vector<double> xy = getXY_Q(cur_s, cur_d);
		vector<double> xy = convert_sd_to_xy(cur_s, cur_d);
		m_next_x_vals.push_back(xy[0]);
		m_next_y_vals.push_back(xy[1]);
		//		cout<<"cur_s, cur_d "<<cur_s<<","<<cur_d<<endl;
	}
	cout<<"consumed="<< consume_num <<", remain="<<m_last_waypoints_num<<"previous_path_x.size()"<<previous_path_x.size()<<endl;
	return true;

}
