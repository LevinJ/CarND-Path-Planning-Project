#!/usr/bin/env python

from ptg import follow_vehicle,follow_goal,keep_lane,LC,calculate_cost,WEIGHTED_COST_FUNCTIONS
from helpers import Vehicle, show_trajectory

def main():
	T = 5.0
	
	
	


	
	
# 	vehicle = Vehicle([1500,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [0, 10, 0]
# 	start_d = [0, 0, 0]
# 	best = keep_lane(start_s, start_d, T, predictions)



# 	vehicle = Vehicle([60,10,0, 2,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [0, 10, 0]
# 	start_d = [2, 0, 0]
# 	best = keep_lane(start_s, start_d, T, predictions)
	
	
	
	
	
	vehicle = Vehicle([150,20,0, 2,0,0])
	vehicle_1 = Vehicle([100,10,0, 6,0,0])
	predictions = {0: vehicle, 1:vehicle_1}
# 	predictions = {0: vehicle}
	start_s = [30, 10, 0]
	start_d = [6, 0, 0]
	best = LC(start_s, start_d, T, predictions, prepare=False, left= False)
#  	
	
	

	best_cost = calculate_cost(best, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
	print("best cost: {}\n\n".format(best_cost))
	
	print("best trajectory {}".format(best))
	show_trajectory(best[0], best[1], best[2], vehicle)
	
	
	

if __name__ == "__main__":
	main()