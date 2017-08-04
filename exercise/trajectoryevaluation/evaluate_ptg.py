#!/usr/bin/env python

from ptg import follow_vehicle,follow_goal,keep_lane,LC,calculate_cost,WEIGHTED_COST_FUNCTIONS
from helpers import Vehicle, show_trajectory

def main():
	T = 5.0
	
	
	


	
	
# 	vehicle = Vehicle([1500,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [125.376, 1.07147, 1.22471]
# 	start_d = [6.14889, -0.0264154, -0.0207073]
# 	best = keep_lane(start_s, start_d, T, predictions)


 
	predictions = {};
	predictions[0] = Vehicle([949.797, 15.2255, 0, 6.02025, 0, 0,]);
	predictions[1] = Vehicle([1031.06, 16.2751, 0, 10.0789, 0, 0,]);
	predictions[2] = Vehicle([887.9, 18.2031, 0, 10.0751, 0, 0,]);
	predictions[3] = Vehicle([913.592, 18.6167, 0, 10.0257, 0, 0,]);
	predictions[4] = Vehicle([876.173, 17.2366, 0, 5.8808, 0, 0,]);
	predictions[5] = Vehicle([962.638, 15.495, 0, 9.99124, 0, 0, ]);
	predictions[6] = Vehicle([937.637, 15.8691, 0, 1.93111, 0, 0,]);
	predictions[7] = Vehicle([943.225, 16.4024, 0, 9.98928, 0, 0]);
	predictions[8] = Vehicle([904.896, 18.4822, 0, 2.27677, 0, 0, ]);
	predictions[9] = Vehicle([844.471, 12.3391, 0, 5.89112, 0, 0,]);
	predictions[10] = Vehicle([874.706, 18.4883, 0, 1.87546, 0, 0,]);
	predictions[11] = Vehicle([959.201, 15.3592, 0, 2.0641, 0, 0,]);
	
	
	start_s = [901.971, 15.1819, 0.0116964]
	start_d = [6, -1.921e-12, 2.20705e-12 ]
	best = keep_lane(start_s, start_d, T, predictions)
	
	
	
	
	
# 	vehicle = Vehicle([150,20,0, 2,0,0])
# 	vehicle_1 = Vehicle([100,10,0, 6,0,0])
# 	predictions = {0: vehicle, 1:vehicle_1}
# # 	predictions = {0: vehicle}
# 	start_s = [30, 10, 0]
# 	start_d = [6, 0, 0]
# 	best = LC(start_s, start_d, T, predictions, prepare=False, left= False)
# #  	
	
	
# 	best_cost = calculate_cost(best, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
# 	print("best cost: {}\n\n".format(best_cost))
	
# 	print("best trajectory {}".format(best))
# 	show_trajectory(best.s_coeff, best.d_coeff, best.t, vehicle)
	
	
	

if __name__ == "__main__":
	main()