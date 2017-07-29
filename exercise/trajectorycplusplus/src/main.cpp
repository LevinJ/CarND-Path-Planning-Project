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



bool close_enough(vector< double > poly, vector<double> target_poly, double eps=0.01) {


	if(poly.size() != target_poly.size())
	{
		cout << "your solution didn't have the correct number of terms" << endl;
		return false;
	}
	for(int i = 0; i < poly.size(); i++)
	{
		double diff = poly[i]-target_poly[i];
		if(abs(diff) > eps)
		{
			cout << "at least one of your terms differed from target by more than " << eps << endl;
			return false;
		}

	}
	return true;
}

struct test_case {

	vector<double> start;
	vector<double> end;
	double T;
};

vector< vector<double> > answers = {{0.0, 10.0, 0.0, 0.0, 0.0, 0.0},{0.0,10.0,0.0,0.0,-0.625,0.3125},{5.0,10.0,1.0,-3.0,0.64,-0.0432}};

std::ostream& operator<< (std::ostream& out, const TrjObject& trj) {
	out<<"s_coeff "<<trj.s_coeff<<endl;
	out<<"d_coeff "<<trj.d_coeff<<endl;
	out<<"t "<<trj.t<<endl;
	out<<"unperturbed_s "<<trj.unperturbed_s<<endl;
	out<<"unperturbed_d "<<trj.unperturbed_d<<endl;
	out<<"unperturbed_t "<<trj.unperturbed_t<<endl;
	out<<"s_goal "<<trj.s_goal<<endl;
	out<<"d_goal "<<trj.d_goal<<endl;

	double t = trj.t;
	vector<double> s = trj.s_coeff;
	vector<double> s_dot = differentiate(s);
	vector<double> s_d_dot = differentiate(s_dot);


	vector<double> S = {to_equation(s, t), to_equation(s_dot,t), to_equation(s_d_dot,t)};
	out<<"S "<<S<<endl;

	vector<double> d = trj.d_coeff;
	vector<double> d_dot = differentiate(d);
	vector<double> d_d_dot = differentiate(d_dot);


	vector<double> D = {to_equation(d, t), to_equation(d_dot,t), to_equation(d_d_dot,t)};
	out<<"D "<<D<<endl;

	return out;
}


int main() {

	Trajectory trj;
	//create test cases
	double T = 5;

	Vehicle vehicle({60,10,0, 2,0,0});
	std::map<int, Vehicle> predictions;
	predictions[0] = vehicle;
	vector<double> start_s = {0, 10, 0};

	vector<double> start_d = {2, 0, 0};
	TrjObject best = trj.keep_lane(start_s, start_d, T, predictions);

	cout<<endl<<"best:"<<endl<<best<<endl;
	double best_cost = trj.calculate_cost(best, predictions, true);
	cout<<"best cost "<<best_cost<<endl;

	return 0;
}
