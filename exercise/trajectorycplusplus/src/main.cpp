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


	vector<double> start_s = {129.132, 6.73314, 7.29003, }; vector<double> start_d = {6.11447, -0.0482957, -0.0123691, };
	predictions[0] = Vehicle({101.831, 21.1753, 0, 2.01398, 0, 0, });
	predictions[1] = Vehicle({275.205, 15.7204, 0, 5.97234, 0, 0, });
	predictions[2] = Vehicle({67.5644, 19.7889, 0, 6.03594, 0, 0, });
	predictions[3] = Vehicle({312.555, 19.2712, 0, 5.56338, 0, 0, });
	predictions[4] = Vehicle({99.4751, 22.1513, 0, 6.00266, 0, 0, });
	predictions[5] = Vehicle({273.109, 18.0787, 0, 9.9603, 0, 0, });
	predictions[6] = Vehicle({300.796, 17.0899, 0, 10.0451, 0, 0, });
	predictions[7] = Vehicle({90.8618, 23.0049, 0, 9.98642, 0, 0, });
	predictions[8] = Vehicle({263.729, 11.197, 0, 5.95275, 0, 0, });
	predictions[9] = Vehicle({56.9801, 23.33, 0, 2.11558, 0, 0, });
	predictions[10] = Vehicle({54.309, 21.0447, 0, 10.11, 0, 0, });
	predictions[11] = Vehicle({292.499, 13.9132, 0, 6.03964, 0, 0, });


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
