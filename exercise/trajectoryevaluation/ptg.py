import numpy as np
import random
from cost_functions import *
from constants import *
import math



class TrjObject(object):

    def __init__(self, s_goal_p, d_goal_p, t_p,
            unperturbed_s_p,unperturbed_d_p, unperturbed_t_p):
        self.s_goal = s_goal_p;
        self.d_goal = d_goal_p;
        self.t = t_p;
        self.unperturbed_s = unperturbed_s_p;
        self.unperturbed_d = unperturbed_d_p;
        self.unperturbed_t = unperturbed_t_p;
        self.baccident = False;
        return
    def __str__(self):
        temp_str = "s_gaol={},d_goal={}, t= {}, \nunperturbed_s={},unperturbed_d={},unperturbed_t={},\
        baccident={}\n".format(self.s_goal, self.d_goal,self.t,self.unperturbed_s,self.unperturbed_d,self.unperturbed_t,self.baccident)
        return temp_str;

    
    
#     std::vector<double> s_goal;
#     std::vector<double> d_goal;
#     std::vector<double> s_coeff;
#     std::vector<double> d_coeff;
#     double t;
#     std::vector<double> unperturbed_s;
#     std::vector<double> unperturbed_d;
#     double unperturbed_t;
#     bool baccident;


# TODO - tweak weights to existing cost functions
WEIGHTED_COST_FUNCTIONS = [
    (time_diff_cost,    0),
    (s_diff_cost,       1),
#     (d_diff_cost,       1),
#     (efficiency_cost,   1),
    (max_jerk_cost,     10),
    (total_jerk_cost,   1),
    (collision_cost,    10),
    (buffer_cost,       1),
    (max_accel_cost,    10),
    (total_accel_cost,  1),
#     (min_speed_cost,    1),
    (exceeds_speed_limit_cost, 10),
    (stays_on_road_cost,    10)
]

m_current_behaviour = ""

def get_lane_num(d):
    return int(d / 4)

    
def LC(start_s, start_d, T, predictions, prepare=True, left= True):
    has_target = False
    s = start_s[0]
    d = start_d[0]
    if left:
        target_lane = get_lane_num(d) + 1
    else:
        target_lane = get_lane_num(d) - 1
    if prepare:
            if left:
                delta = [0, 0,0,-LANE_WIDTH,0,0]
            else:
                delta = [0, 0,0,LANE_WIDTH,0,0]
    else:
        delta = [0, 0,0,0,0,0]
            
    all_vehicles = [(v_id, v) for (v_id, v) in predictions.items() if get_lane_num(v.start_state[3]) == target_lane]
    if len(all_vehicles) != 0 :
        closetest = min(all_vehicles, key=lambda v: abs(v[1].start_state[0] - s))
        closetest_id = closetest[0]
        closetest = closetest[1].start_state
        
        distance = abs(closetest[0] - s)
        if closetest[0] < s:
            max_distance = closetest[1] * T
            delta_s = SAFE_DISTANCE_BUFFER
        else:
            max_distance = (SPEED_LIMIT - closetest[1])* T
            delta_s = -SAFE_DISTANCE_BUFFER
        if distance < max_distance:
            has_target = True
    if has_target:
        
        target_vehicle = closetest_id
        if not prepare:
            delta[0] = delta_s
        
        return follow_vehicle(start_s, start_d, T, target_vehicle, delta,  predictions)
    #no target
    if prepare:
        return keep_lane(start_s, start_d, T, predictions)
    
    #lane change
    goal_s = [s+ (SPEED_LIMIT + start_s[1])*T/2, SPEED_LIMIT, 0]
    goal_d = [target_lane*LANE_WIDTH + LANE_WIDTH/2,0,0]
    return follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions)
    
             
def keep_lane(start_s, start_d, T, predictions):
    s = start_s[0];
    d = start_d[0];
    leading_id = -1;
    distance = math.inf;
    #Find leading vehicle in the same lane
    for k, v in predictions.items():
        if get_lane_num(v.start_state[3]) != get_lane_num(d) or v.start_state[0] < s:
            continue;
        
        if (v.start_state[0] -s)< distance:
            distance = v.start_state[0] -s;
            leading_id = k;
        
    
    if leading_id !=-1:
        #if we have leading vehicle, check whether it's within safe distance
        cur_distance = predictions[leading_id].start_state[0] - start_s[0];
        if cur_distance < SAFE_DISTANCE_BUFFER:
           
            #make sure we do not change lanes
            delta_d = get_lane_dist(get_lane_num(start_d[0])) - predictions[leading_id].start_state[3];
            delta = [0, 0,0,delta_d,0,0];
            print("trj: keep lane, has target " + str(leading_id) + ", target vehicle state=" + str(predictions[leading_id].start_state));

            trjobj =  follow_vehicle(start_s, start_d, T, leading_id, delta,  predictions);
            print("old distance=" + str(cur_distance) + ", new distance=" + str((predictions[leading_id].state_in(T)[0] - trjobj.s_goal[0])))
            return trjobj;
        
    

    #follow a reasonably set goal
    avg_speed = (SPEED_LIMIT + start_s[1])/2;
    if avg_speed > SPEED_LIMIT:
        avg_speed = SPEED_LIMIT;
    
    print("trj: keep lane, no target, " + str(avg_speed))
    goal_s = [s+ avg_speed*T, SPEED_LIMIT, 0];


    goal_d = [get_lane_dist(get_lane_num(start_d[0])),0,0];
    trjobj =  follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions)
    print("planned T=" + ", actual T=" + str(trjobj.t))
    return trjobj;
#     has_target = False
#     s = start_s[0]
#     d = start_d[0]
#     in_front = [(v_id, v) for (v_id, v) in predictions.items() if get_lane_num(v.start_state[3]) == get_lane_num(d) and v.start_state[0] > s ]
#     
#     
#     
#     if len(in_front)>0:
#     
#         leading = min(in_front, key=lambda v: v[1].start_state[0] - s)
#         leading_id = leading[0]
#         leading = leading[1].start_state
#         
#         deta_distance = leading[0] - s
#         
#         if deta_distance < SAFE_DISTANCE_BUFFER + 5:
#             #let's increase the gap in a stable/gradual fashion
#             deta_distance = deta_distance + 10;
#             if(deta_distance >SAFE_DISTANCE_BUFFER):
#                 deta_distance = SAFE_DISTANCE_BUFFER;
#             deta_distance = 4
#             
#             #make sure we do not change lanes
#             delta_d = get_lane_dist(get_lane_num(start_d[0])) - predictions[leading_id].start_state[3];
#             delta = [-deta_distance, 0,0,delta_d,0,0];
#             print("trj: keep lane, has target " + str(leading_id) + " delta, "+str(delta));
# 
#             return follow_vehicle(start_s, start_d, T, leading_id, delta,  predictions);
#     
#    
#     #follow a reasonably set goal
#     avg_speed = (SPEED_LIMIT + start_s[1])/2;
#     if avg_speed > SPEED_LIMIT:
#         avg_speed = SPEED_LIMIT
#     
#     print("trj: keep lane, no target, "+ str(avg_speed));
#     goal_s = [s+ avg_speed*T, SPEED_LIMIT, 0];
# 
# 
#     goal_d = [get_lane_dist(get_lane_num(start_d[0])),0,0];
#     return follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions);
        
        

def get_lane_dist(lane_id):
    return lane_id*LANE_WIDTH + LANE_WIDTH/2;
   
def follow_vehicle(start_s, start_d, T, target_vehicle, delta,  predictions):
    
    all_goals = perturb_goals(start_s, start_d, T, None, None, target_vehicle, delta,predictions)
    return PTG(start_s, start_d,all_goals, T,predictions)

def perturb_goals(start_s, start_d, T, goal_s, goal_d, target_vehicle, delta,predictions):
    
    delta_s_distances=[]
    
    min_dist = COLLISION_DISTANCE
    max_dist = SAFE_DISTANCE_BUFFER + 30
    d_s_dist = 5

    s_distance = min_dist;
    while s_distance < max_dist:
        delta_s_distances.append(-s_distance)
        s_distance += d_s_dist
    
    


    all_goals = []

    if target_vehicle == -1:
        #if we do not follow any vehicle, we purturb the t only
        #this is because we may not be able to reach speed limit with given T
        timestep = 0.5
        t = T - 4 * timestep
        while t <= T + 4 * timestep:
            all_goals.push_back(TrjObject(goal_s,goal_d,t,goal_s,goal_d, T))
            t = t + timestep
        

        return all_goals;
    


    target = predictions[target_vehicle]
    target_vehicle_state = target.state_in(T)

    for delta_distace in delta_s_distances:



        purturbed_goal_s = [target_vehicle_state[0] + delta_distace, target_vehicle_state[1]+delta[1],0];
        if purturbed_goal_s[0] < 0:
            #s should always be positve number
            purturbed_goal_s[0] = 5;
        
        if purturbed_goal_s[1] > SPEED_LIMIT:
            #make sure we do not exceed speed limit when following vehicles
            purturbed_goal_s[1] = SPEED_LIMIT
        
        purturbed_goal_d = [target_vehicle_state[3] + delta[3],
                target_vehicle_state[4]+delta[4],target_vehicle_state[5]+delta[5]]
        #the only we purturbed here is the s
        goal_s= [target_vehicle_state[0] - SAFE_DISTANCE_BUFFER, purturbed_goal_s[1], purturbed_goal_s[2]];
        goal_d = purturbed_goal_d;
        all_goals.append(TrjObject(purturbed_goal_s,purturbed_goal_d,T,goal_s,goal_d, T));
    
    return all_goals;
#     all_goals = []
#     timestep = 0.5
#     t = T - 4 * timestep
#     
#     if goal_s is None:
#         folllow_vehicle = True
#     else:
#         folllow_vehicle = False
#         
#     while t <= T + 4 * timestep:
#         
#         if folllow_vehicle:
#             target = predictions[target_vehicle]
#             target_state = np.array(target.state_in(t)) + np.array(delta)
#             goal_s = target_state[:3]
#             goal_d = target_state[3:]
#         
#         all_goals.append((goal_s,goal_d, t,goal_s,goal_d))
#         print("t {}, goal {} {}, unperturbed {} {}".format(t, goal_s, goal_d, goal_s, goal_d))
#         for _ in range(N_SAMPLES):
#             perturbed = perturb_goal(goal_s, goal_d)
#            
#             all_goals.append((perturbed[0], perturbed[1], t,goal_s,goal_d))
#         t += timestep
#         
        
    
#     return all_goals

def follow_goal(start_s, start_d, T, goal_s, goal_d,  predictions):
    all_goals = perturb_goals(start_s, start_d, T, goal_s, goal_d, None, None,predictions)
    return PTG(start_s, start_d,all_goals, T,predictions)
   


    
def PTG(start_s, start_d,all_trjs, T,predictions):
    """
    Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).

    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that 
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at 
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal 
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a 
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method 
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/ 
     this trajectory.
    """
    min_cost =  math.inf;
    count = 1
    best_count = 0;
    for trj in all_trjs:
        print("trj {}".format(count))
        count = count + 1
        trj.s_coeff = JMT(start_s, trj.s_goal, trj.t);
        trj.d_coeff = JMT(start_d, trj.d_goal, trj.t);

        cost = calculate_cost(trj, predictions, WEIGHTED_COST_FUNCTIONS);
        if(cost < min_cost):
            best =trj;
            min_cost = cost;
            best_count = count;
        
        
    
    print("\n#####best trjid={}, min cost = {}, \nbest trj={}".format(best_count, min_cost, best))
    if min_cost >=10:
        best.baccident = True
    else:
        best.baccident = False

    
    return best;

    
    # generate alternative goals
    
    
    # find best trajectory
#     trajectories = []
#     for goal in all_goals:
#         s_goal, d_goal, t, unperturbed_s,unperturbed_d = goal
#         s_coefficients = JMT(start_s, s_goal, t)
# #         print("start_d: {}, d_goal{}".format(start_d, d_goal))
#         d_coefficients = JMT(start_d, d_goal, t)
#         trajectories.append(tuple([s_coefficients, d_coefficients, t, unperturbed_s,unperturbed_d, T]))
#     
#     best = min(trajectories, key=lambda tr: calculate_cost(tr, predictions, WEIGHTED_COST_FUNCTIONS))
#     
#     return best
    

def calculate_cost(trajectory,  predictions, cost_functions_with_weights, verbose=False):
    cost = 0
    verbose = True
    for cf, weight in cost_functions_with_weights:
        new_cost = weight * cf(trajectory, predictions)
        cost += new_cost
        if verbose:
            print("cost for {} is \t {}".format(cf.__name__, new_cost))
    print("trajectory {}, cost {}".format(trajectory , cost))
    return cost

def perturb_goal(goal_s, goal_d):
    """
    Returns a "perturbed" version of the goal.
    """
    new_s_goal = []
    for mu, sig in zip(goal_s, SIGMA_S):
        new_s_goal.append(random.gauss(mu, sig))
    new_s_goal[2 ]= goal_s[2]

    new_d_goal = []
    new_d_goal = goal_d
#     for mu, sig in zip(goal_d, SIGMA_D):
#         new_d_goal.append(random.gauss(mu, sig))
        
    return tuple([new_s_goal, new_d_goal])

def JMT(start, end, T):
    """
    Calculates Jerk Minimizing Trajectory for start, end and T.
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T**2
    c_1 = a_1 + 2* a_2 * T
    c_2 = 2 * a_2
    
    A = np.array([
            [  T**3,   T**4,    T**5],
            [3*T**2, 4*T**3,  5*T**4],
            [6*T,   12*T**2, 20*T**3],
        ])
    B = np.array([
            end[0] - c_0,
            end[1] - c_1,
            end[2] - c_2
        ])
    a_3_4_5 = np.linalg.solve(A,B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas