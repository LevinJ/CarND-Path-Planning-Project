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

	vector<double> start_s = {190.992, 16.6882, -0.826488, }; vector<double> start_d = {8.42264, 0.996988, -0.243736, };
	predictions[0] = Vehicle({119.911, 6.8214, 0, 6.00479, 0, 0, });
	predictions[1] = Vehicle({357.255, 16.9804, 0, 1.8634, 0, 0, });
	predictions[2] = Vehicle({383.607, 15.7757, 0, 9.94134, 0, 0, });
	predictions[3] = Vehicle({153.381, 15.0525, 0, 5.89386, 0, 0, });
	predictions[4] = Vehicle({349.957, 15.971, 0, 9.6372, 0, 0, });
	predictions[5] = Vehicle({392.405, 18.115, 0, 5.96076, 0, 0, });
	predictions[6] = Vehicle({160.88, 20.7008, 0, 10.2156, 0, 0, });
	predictions[7] = Vehicle({166.036, 20.4498, 0, 2.54657, 0, 0, });
	predictions[8] = Vehicle({189.804, 21.0625, 0, 2.40076, 0, 0, });
	predictions[9] = Vehicle({354.031, 15.9978, 0, 5.77151, 0, 0, });
	predictions[10] = Vehicle({121.265, 18.3557, 0, 2.01142, 0, 0, });
	predictions[11] = Vehicle({325.638, 15.8803, 0, 1.38752, 0, 0, });

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
