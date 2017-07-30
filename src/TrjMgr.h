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

class TrjMgr {
public:
	TrjMgr();
	void generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
			const std::vector<std::vector<double>> &sensor_fusion);

	virtual ~TrjMgr();
	std::vector<double> m_next_x_vals;
	std::vector<double> m_next_y_vals;
	vector<double> m_maps_s;
	vector<double> m_maps_x;
	vector<double> m_maps_y;
private:
	std::vector<std::vector<double>> get_start_state(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
	std::map<int, Vehicle> get_predictons(const std::vector<std::vector<double>> &sensor_fusion);
	void convert_next_waypoints(const TrjObject &trjobj);
	Trajectory m_trajectory;

	vector<double> getXY_Q(double s, double d);
	int m_last_waypoints_num;
	TrjObject m_last_trjobj;
	std::vector<std::vector<double>> process_prevpath(const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
	vector<double> getFrenet_Q(double x, double y, double theta);

};

#endif /* SRC_TRJMGR_H_ */
