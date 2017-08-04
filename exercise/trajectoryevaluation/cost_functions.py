from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np
# COST FUNCTIONS
def time_diff_cost(traj, predictions):
    """
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
    """
    t = traj.t;
    unperturbed_t = traj.unperturbed_t;
    return logistic(float(abs(t-unperturbed_t)) / unperturbed_t)

def s_diff_cost(traj, predictions):
    """
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
    """
    s, _, t, unperturbed_s,_,_ = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    
    
    S = [f(t) for f in get_f_and_N_derivatives(s, 2)]
    cost = 0
    for actual, expected, sigma in zip(S, unperturbed_s, SIGMA_S):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

def d_diff_cost(traj, predictions):
    """
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
    """
    _, d_coeffs, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    
    d_dot_coeffs = differentiate(d_coeffs)
    d_ddot_coeffs = differentiate(d_dot_coeffs)

    d = to_equation(d_coeffs)
    d_dot = to_equation(d_dot_coeffs)
    d_ddot = to_equation(d_ddot_coeffs)

    D = [d(t), d_dot(t), d_ddot(t)]
    
    cost = 0
    for actual, expected, sigma in zip(D, unperturbed_d, SIGMA_D):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

def collision_cost(traj, predictions):
    """
    Binary cost function which penalizes collisions.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    if nearest < 2*VEHICLE_RADIUS: 
        return 1.0
    else: 
        return 0.0

def buffer_cost(traj, predictions):
    """
    Penalizes getting close to other vehicles.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    return logistic(2*VEHICLE_RADIUS / nearest)
    
def stays_on_road_cost(traj, predictions):
    _, d_coeffs, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    
    d = to_equation(d_coeffs)
    all_ds = [d(float(t)/100 * i) for i in range(100)]
    
    if max(all_ds) <= MAX_D and min(all_ds) >=MIN_D:
        return 0
    else:
        return 1
    

def exceeds_speed_limit_cost(traj, predictions):
    s, _, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    speed = differentiate(s)
   
    speed = to_equation(speed)
    all_speeds = [speed(float(t)/100 * i) for i in range(100)]
    max_speed = max(all_speeds)
    min_speed = min(all_speeds)
    
    if max_speed <= SPEED_LIMIT and min_speed>= MIN_SPEED:
        return 0
    else:
        print("max_v={}, min_v={}".format(max_speed, min_speed))
        return 1
    


def efficiency_cost(traj, predictions):
    """
    Rewards high average speeds.
    """
    s, _, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    s = to_equation(s)
    avg_v = float(s(t)) / t
    targ_s, _, _, _, _, _ = predictions[target_vehicle].state_in(t)
    targ_v = float(targ_s) / t
    return logistic(2*float(targ_v - avg_v) / avg_v)

def total_accel_cost(traj,  predictions):
    s, d, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    total_acc = 0
    dt = float(t) / 100.0
    for i in range(100):
        cur_t = dt * i
        acc = a(cur_t)
        total_acc += abs(acc*dt)
    acc_per_second = total_acc / cur_t
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )
    
def max_accel_cost(traj, predictions):
    s, d, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(t)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0
    

def max_jerk_cost(traj, predictions):
    s, d, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
    jerk = to_equation(jerk)
    all_jerks = [jerk(float(t)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > MAX_JERK: 
        print("max_jerk={}".format(max_jerk))
        return 1
    else: return 0

def total_jerk_cost(traj, predictions):
    s, d, t, unperturbed_s,unperturbed_d,unperturbed_t = traj.s_coeff, traj.d_coeff, traj.t, traj.unperturbed_s, traj.unperturbed_d, traj.unperturbed_t
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = to_equation(differentiate(s_d_dot))
    total_jerk = 0
    dt = float(t) / 100.0
    for i in range(100):
        cur_t = dt * i
        j = jerk(cur_t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / cur_t
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )

