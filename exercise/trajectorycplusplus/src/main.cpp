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

	vector<double> start_s = {6933.3, 7.9119, -5.04495, }; vector<double> start_d = {2, 4.02057e-16, -1.14638e-17, };
	predictions[0] = Vehicle({7005.42, 16.2835, 0, 2.09041, 0, 0, });
	predictions[1] = Vehicle({6949.54, 15.3966, 0, 6.10177, 0, 0, });
	predictions[2] = Vehicle({6910.25, 5.21082, 0, 2.29903, 0, 0, });
	predictions[3] = Vehicle({104.722, 17.3744, 0, 6.00703, 0, 0, });
	predictions[4] = Vehicle({6875.11, 17.9586, 0, 1.96471, 0, 0, });
	predictions[5] = Vehicle({6895.92, 9.65587, 0, 2.87882, 0, 0, });
	predictions[6] = Vehicle({6871.69, 17.7541, 0, 6.05127, 0, 0, });
	predictions[7] = Vehicle({6974.01, 16.3704, 0, 5.94928, 0, 0, });
	predictions[8] = Vehicle({6992.92, 16.6258, 0, 10.0224, 0, 0, });
	predictions[9] = Vehicle({6923.18, 21.394, 0, 6.22278, 0, 0, });
	predictions[10] = Vehicle({76.0971, 14.9739, 0, 9.99647, 0, 0, });
	predictions[11] = Vehicle({100.122, 16.5653, 0, 9.99908, 0, 0, });


	TrjObject best = trj.keep_lane(start_s, start_d, T, predictions);
//	TrjObject best = trj.LC(start_s, start_d, T, predictions, true);


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
