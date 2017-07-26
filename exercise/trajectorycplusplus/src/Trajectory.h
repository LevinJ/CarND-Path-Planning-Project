/*
 * Trajectory.h
 *
 *  Created on: Jul 24, 2017
 *      Author: levin
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>

class Trajectory {
public:
	Trajectory();
	virtual ~Trajectory();
	std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);

};

#endif /* TRAJECTORY_H_ */
