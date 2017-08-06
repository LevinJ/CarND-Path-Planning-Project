/*
 * Constants.h
 *
 *  Created on: Jul 26, 2017
 *      Author: levin
 */

#ifndef EXERCISE_TRAJECTORYCPLUSPLUS_SRC_CONSTANTS_H_
#define EXERCISE_TRAJECTORYCPLUSPLUS_SRC_CONSTANTS_H_

const int N_SAMPLES = 0;
const float SIGMA_S[] = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
const float SIGMA_D[] = {1.0, 1.0, 1.0};
const float SIGMA_T = 2.0;

//const float MAX_JERK = 10;// # m/s/s/s
//const float MAX_ACCEL= 10;// # m/s/s


//for the sake of safe planning
const float MAX_JERK = 9;// # m/s/s/s
const float MAX_ACCEL= 9;// # m/s/s


const float MIN_SPEED = 0;

const float EXPECTED_JERK_IN_ONE_SEC = 2;// # m/s/s
const float EXPECTED_ACC_IN_ONE_SEC = 1;// # m/s

const double HARD_SPEED_LIMIT = 22.352; // 50mph in m/s
const float SPEED_LIMIT = 21.3;
const float VEHICLE_RADIUS = 1.5;// # model vehicle as circle to simplify collision detection
const float COLLISION_DISTANCE = 3;
//for the sake of safe driving, we should apply 3 seconds rules
const float SAFE_DISTANCE_BUFFER = 3*SPEED_LIMIT;

const float MAX_D = 12;//maximum lateral distance
const float MIN_D = 0;

const float LANE_WIDTH = 4;

const int REUSE_PREV_POINTS_NUM = 10;
const double FRAME_UPDATE_TIME = 0.02;
const int All_POINTS_NUM = 150;

const double MAX_S = 6945.554;

const double FRONT_GAP_THRESH = 50.0;
const double BACK_GAP_THRESH = 15;

//we want the car to stay in a lane for at least LAST_LC_ELAPSED_COST_THRES seconds.
const double LAST_LC_ELAPSED_COST_THRES = 6000; //milliseconds



#endif /* EXERCISE_TRAJECTORYCPLUSPLUS_SRC_CONSTANTS_H_ */
