#!/usr/bin/env python

import numpy as np
from quintic import JMT
from matplotlib import pyplot as plt

TEST_CASES = [
    ([0, 10, 0], [10, 10, 0], 1),
    ([0, 10, 0], [20, 15, 20], 2),
    ([5, 10, 2], [-30, -20, -4], 5),
]

ANSWERS = [[0.0, 10.0, 0.0, 0.0, -0.0, 0.0],
 [0.0,
  10.0,
  0.0,
  -1.2096774193548387,
  0.58467741935483875,
  0.010080645161290322],
 [5.0,
  10.0,
  1.0,
  -2.126970954356846,
  0.25078838174273854,
  -0.0002788381742738588]]

def close_enough(poly, target_poly, eps=0.01):
	if type(target_poly) != type(poly):
		target_poly = list(target_poly)
	if len(poly) != len(target_poly):
		print("your solution didn't have the correct number of terms")
		return False
	for term, target_term in zip(poly, target_poly):
		if abs(term-target_term) > eps:
			print("at least one of your terms differed from target by more than {}".format(eps))
			return False
	return True
    
def to_equation(coefficients):
    """
    Takes the coefficients of a polynomial and creates a function of
    time from them.
    """
    def f(t):
        total = 0.0
        for i, c in enumerate(coefficients): 
            total += c * t ** i
        return total
    return f

def show_trajectory(s_coeffs, T):
    s = to_equation(s_coeffs)
  
    t_list = []
    s_list = []
    t = 0
    while t <= T+0.01:
        s_list.append(s(t))
        t_list.append(t)
        t += 0.25
    plt.figure()
    plt.plot(t_list, s_list, label="sdc")

    plt.legend()
#     plt.show()
def differentiate(coefficients):
    """
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
    """
    new_cos = []
    for deg, prev_co in enumerate(coefficients[1:]):
        new_cos.append((deg+1) * prev_co)
    return new_cos
SPEED_LIMIT = 22
    
def exceeds_speed_limit_cost(s, T):
    
    
    speed = differentiate(s)
   
    speed = to_equation(speed)
#     all_speeds = [speed(float(t)/100 * i) for i in range(100)]
    all_speeds = []
    t_list = []
    t = 0
    while t <= T+0.01:
        all_speeds.append(speed(t))
        t_list.append(t)
        t += 0.02
    
    max_speed = max(all_speeds)
    min_speed = min(all_speeds)
    
    if max_speed <= SPEED_LIMIT and min_speed>= MIN_SPEED:
        print("ok speed")
    else:
        print("max_v={}, min_v={}".format(max_speed, min_speed))
    plt.figure()
    plt.plot(t_list, all_speeds, label="speed")
    plt.legend()
#     plt.show()
    
 
def main():
    
    T = 5
    start = [124.834, 0, 0]
    end = [237.334, 22, 0]
    jmt = JMT(start, end, T)
    print(jmt)
    exceeds_speed_limit_cost(jmt, T)
    show_trajectory(jmt, T)
    plt.show()
# 	for test_case, answer in zip(TEST_CASES, ANSWERS):
# 		start, end, T = test_case
# 		jmt = JMT(start, end, T)
# 		correct = close_enough(jmt, answer)
# 		if not correct:
# 			print("try again!")
# 			return False
# 	print("Nice work!")
# 	return True

if __name__ == "__main__":
	main()