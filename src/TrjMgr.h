/*
 * TrjMgr.h
 *
 *  Created on: Jul 30, 2017
 *      Author: levin
 */

#ifndef SRC_TRJMGR_H_
#define SRC_TRJMGR_H_

#include <vector>
#include "Trajectory.h"
#include "Behavior.h"

class TrjMgr {
public:
	TrjMgr(const vector<double> &maps_s, const vector<double> &maps_x, vector<double> &maps_y,
			const vector<double> &maps_dx, const vector<double> &maps_dy);
	void generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
			const std::vector<std::vector<double>> &sensor_fusion);

	virtual ~TrjMgr();
	std::vector<double> m_next_x_vals;
	std::vector<double> m_next_y_vals;
	vector<double> m_maps_s;
	vector<double> m_maps_x;
	vector<double> m_maps_y;
	vector<double> m_maps_dx;
	vector<double> m_maps_dy;
private:
	bool handle_lane_change(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
			const std::vector<std::vector<double>> &sensor_fusion);
	vector<double> convert_sd_to_xy(const double s, const double d);

	vector<vector<double>> m_last_waypoints_s;
	vector<vector<double>> m_last_waypoints_d;
	std::vector<std::vector<double>> get_start_state(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
	std::map<int, Vehicle> get_predictons(const std::vector<std::vector<double>> &sensor_fusion, double start_s,const std::vector<double> &car_state);
	void convert_next_waypoints(const TrjObject &trjobj);
	Trajectory m_trajectory;

	vector<double> getXY_Q(double s, double d);
	int m_last_waypoints_num;
//	TrjObject m_last_trjobj;
	std::vector<std::vector<double>> process_prevpath(const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
	vector<double> getFrenet_Q(double x, double y, double theta);
	Behavior m_behavior;
	BehvStates m_current_sate;

};

#endif /* SRC_TRJMGR_H_ */
