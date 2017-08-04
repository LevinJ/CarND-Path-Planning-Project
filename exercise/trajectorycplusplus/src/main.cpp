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
	predictions[3] = Vehicle({4914.85, 17.3248, 0, 6.02401, 0, 0, });


//	predictions[0] = Vehicle({5086.83, 20.0965, 0, 10.3737, 0, 0, });
//	predictions[1] = Vehicle({4783.18, 20.9676, 0, 6.19801, 0, 0, });
//	predictions[2] = Vehicle({4902.15, 18.6779, 0, 9.67319, 0, 0, });
//	predictions[3] = Vehicle({4914.85, 17.3248, 0, 6.02401, 0, 0, });
//	predictions[4] = Vehicle({4750.67, 17.8621, 0, 2.39262, 0, 0, });
//	predictions[5] = Vehicle({4775.35, 16.7066, 0, 2.12721, 0, 0, });
//	predictions[6] = Vehicle({4849.37, 21.3008, 0, 6.30517, 0, 0, });
//	predictions[7] = Vehicle({4933.6, 17.1547, 0, 6.56412, 0, 0, });
//	predictions[8] = Vehicle({4873.87, 18.81, 0, 6.11818, 0, 0, });
//	predictions[9] = Vehicle({4798.29, 16.5046, 0, 2.20831, 0, 0, });
//	predictions[10] = Vehicle({4819.7, 20.1545, 0, 6.00302, 0, 0, });



	vector<double> start_s = {4899.11, 19.1133, -0.546801}; vector<double> start_d = {6, 1.74294e-15, 1.48012e-17};

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
