/*
 * Constants.h
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#ifndef EXERCISE_TRAJECTORYCPLUSPLUS_SRC_CONSTANTS_H_
#define EXERCISE_TRAJECTORYCPLUSPLUS_SRC_CONSTANTS_H_

const int N_SAMPLES = 10;
const float SIGMA_S[] = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
const float SIGMA_D[] = {1.0, 1.0, 1.0};
const float SIGMA_T = 2.0;

const float MAX_JERK = 10;// # m/s/s/s
const float MAX_ACCEL= 10;// # m/s/s

const float MIN_SPEED = 0;

const float EXPECTED_JERK_IN_ONE_SEC = 2;// # m/s/s
const float EXPECTED_ACC_IN_ONE_SEC = 1;// # m/s

const float SPEED_LIMIT = 25;
const float VEHICLE_RADIUS = 1.5;// # model vehicle as circle to simplify collision detection
const float SAFE_DISTANCE_BUFFER = 2*VEHICLE_RADIUS;

const float MAX_D = 12;//maximum lateral distance
const float MIN_D = 0;

const float LANE_WIDTH = 4;

const int REUSE_PREV_POINTS_NUM = 50;
const double FRAME_UPDATE_TIME = 0.02;



#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_CONSTANTS_H_ */
