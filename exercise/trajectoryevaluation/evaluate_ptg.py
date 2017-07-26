#!/usr/bin/env python

from ptg import follow_vehicle,follow_goal,keep_lane,PLC
from helpers import Vehicle, show_trajectory

def main():
	T = 5.0
	
	
	
	#keep lane, follow vehicle
# 	vehicle = Vehicle([50,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	target = 0
# 	delta = [-3, 0, 0, 0, 0 ,0]
# 	start_s = [0, 10, 0]
# 	start_d = [0, 0, 0]
# 	best = follow_vehicle(start_s, start_d, T, target, delta, predictions)
	
	
	#keep lane,follow goal
	
# 	vehicle = Vehicle([1500,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [0, 10, 0]
# 	start_d = [0, 0, 0]
# 	goal_s = [130,30,0]
# 	goal_d = [0,0,0]

	
	
# 	vehicle = Vehicle([1500,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [0, 10, 0]
# 	start_d = [0, 0, 0]
# 	best = keep_lane(start_s, start_d, T, predictions)



# 	vehicle = Vehicle([50,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [0, 10, 0]
# 	start_d = [0, 0, 0]
# 	best = keep_lane(start_s, start_d, T, predictions)
	
	
	
	#PLCL, with target
# 	vehicle = Vehicle([50,20,0, 6,0,0])
# 	vehicle_1 = Vehicle([50,10,0, 2,0,0])
# 	predictions = {0: vehicle, 1:vehicle_1}
# 	start_s = [30, 10, 0]
# 	start_d = [2, 0, 0]
# 	best = PLCL(start_s, start_d, T, predictions)
	
	
	#PLCR, with target
	vehicle = Vehicle([50,20,0, 2,0,0])
	vehicle_1 = Vehicle([50,10,0, 6,0,0])
	predictions = {0: vehicle, 1:vehicle_1}
	start_s = [30, 10, 0]
	start_d = [6, 0, 0]
	best = PLC(start_s, start_d, T, predictions, PLCL=False)
 	
	
	

	
	print("best trajectory {}".format(best))
	show_trajectory(best[0], best[1], best[2], vehicle)
	

if __name__ == "__main__":
	main()