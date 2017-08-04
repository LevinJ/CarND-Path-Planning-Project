#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen/Dense"
#include "Trajectory.h"
#include "Helper.h"
#include <iterator>
#include "TrjCost.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


int main() {

	Trajectory trj;
	//create test cases
	double T = 5;


	//keep lane

	std::map<int, Vehicle> predictions;
	predictions[0] = Vehicle({949.797, 15.2255, 0, 6.02025, 0, 0,});
//	predictions[1] = Vehicle({1031.06, 16.2751, 0, 10.0789, 0, 0,});
//	predictions[2] = Vehicle({887.9, 18.2031, 0, 10.0751, 0, 0,});
//	predictions[3] = Vehicle({913.592, 18.6167, 0, 10.0257, 0, 0,});
//	predictions[4] = Vehicle({876.173, 17.2366, 0, 5.8808, 0, 0,});
//	predictions[5] = Vehicle({962.638, 15.495, 0, 9.99124, 0, 0, });
//	predictions[6] = Vehicle({937.637, 15.8691, 0, 1.93111, 0, 0,});
//	predictions[7] = Vehicle({943.225, 16.4024, 0, 9.98928, 0, 0});
//	predictions[8] = Vehicle({904.896, 18.4822, 0, 2.27677, 0, 0, });
//	predictions[9] = Vehicle({844.471, 12.3391, 0, 5.89112, 0, 0,});
//	predictions[10] = Vehicle({874.706, 18.4883, 0, 1.87546, 0, 0,});
//	predictions[11] = Vehicle({959.201, 15.3592, 0, 2.0641, 0, 0,});



	vector<double> start_s = {901.971, 15.1819, 0.0116964};
	vector<double> start_d = {6, -1.921e-12, 2.20705e-12};
	TrjObject best = trj.keep_lane(start_s, start_d, T, predictions);


	//lane change
//	Vehicle vehicle({150,20,0, 2,0,0});
//	Vehicle vehicle_1 = Vehicle({100,10,0, 6,0,0});
//	std::map<int, Vehicle> predictions;
//	predictions[0] = vehicle;
//	predictions[1] = vehicle_1;

//	vector<double> start_s = {30, 10, 0};
//	vector<double> start_d = {6, 0, 0};
//	TrjObject best = trj.LC(start_s, start_d, T, predictions, false, false);

//	cout<<endl<<"best:"<<endl<<best<<endl;
//	double best_cost = trj.calculate_cost(best, predictions, true);
//	cout<<"best cost "<<best_cost<<endl;

	return 0;
}
