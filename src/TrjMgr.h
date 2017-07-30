/*
 * TrjMgr.h
 *
 *  Created on: Jul 30, 2017
 *      Author: levin
 */

#ifndef SRC_TRJMGR_H_
#define SRC_TRJMGR_H_

#include <vector>

class TrjMgr {
public:
	TrjMgr();
	void generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
			const std::vector<std::vector<double>> &sensor_fusion);

	virtual ~TrjMgr();
	std::vector<double> m_next_x_vals;
	std::vector<double> m_next_y_vals;
};

#endif /* SRC_TRJMGR_H_ */
