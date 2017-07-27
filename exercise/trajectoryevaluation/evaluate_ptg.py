#!/usr/bin/env python

from ptg import follow_vehicle,follow_goal,keep_lane,LC
from helpers import Vehicle, show_trajectory

def main():
	T = 5.0
	
	
	


	
	
# 	vehicle = Vehicle([1500,10,0, 0,0,0])
# 	predictions = {0: vehicle}
# 	start_s = [0, 10, 0]
# 	start_d = [0, 0, 0]
# 	best = keep_lane(start_s, start_d, T, predictions)



	vehicle = Vehicle([50,10,0, 2,0,0])
	predictions = {0: vehicle}
	start_s = [0, 10, 0]
	start_d = [2, 0, 0]
	best = keep_lane(start_s, start_d, T, predictions)
	
	
	
	
	
# 	vehicle = Vehicle([50,20,0, 2,0,0])
# 	vehicle_1 = Vehicle([50,10,0, 6,0,0])
# 	predictions = {0: vehicle, 1:vehicle_1}
# # 	predictions = {0: vehicle}
# 	start_s = [30, 10, 0]
# 	start_d = [6, 0, 0]
# 	best = LC(start_s, start_d, T, predictions, prepare=False, left= False)
#  	
	
	

	
	print("best trajectory {}".format(best))
	show_trajectory(best[0], best[1], best[2], vehicle)
	

if __name__ == "__main__":
	main()