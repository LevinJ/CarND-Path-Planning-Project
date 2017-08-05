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

bool g_debugtrj = true;
int main() {

	Trajectory trj;
	//create test cases
	double T = 5;


	//keep lane

	std::map<int, Vehicle> predictions;

	vector<double> start_s = {125.014, 0.706152, 1.42045, }; vector<double> start_d = {6.16221, -0.0106847, -0.0243536, };
	predictions[0] = Vehicle({45.6794, 21.5652, 0, 6.09133, 0, 0, });
	predictions[1] = Vehicle({256.727, 17.9005, 0, 1.96267, 0, 0, });
	predictions[2] = Vehicle({73.0529, 22.8169, 0, 1.98959, 0, 0, });
	predictions[3] = Vehicle({285.815, 17.7483, 0, 10.0259, 0, 0, });
	predictions[4] = Vehicle({285.402, 20.6783, 0, 2.0232, 0, 0, });
	predictions[5] = Vehicle({251.178, 21.0037, 0, 9.98537, 0, 0, });
	predictions[6] = Vehicle({285.049, 46.2814, 0, 9.99988, 0, 0, });
	predictions[7] = Vehicle({70.4752, 52.1038, 0, 6.00001, 0, 0, });

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
