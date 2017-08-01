N_SAMPLES = 10
SIGMA_S = [10.0, 4.0, 2.0] # s, s_dot, s_double_dot
SIGMA_D = [1.0, 1.0, 1.0]
SIGMA_T = 2.0

MAX_JERK = 10 # m/s/s/s
MAX_ACCEL= 10 # m/s/s

MIN_SPEED = 0

EXPECTED_JERK_IN_ONE_SEC = 10 # m/s/s
EXPECTED_ACC_IN_ONE_SEC = 10 # m/s

SPEED_LIMIT = 22
VEHICLE_RADIUS = 1.5 # model vehicle as circle to simplify collision detection
SAFE_DISTANCE_BUFFER = 2*SPEED_LIMIT

MAX_D = 12
MIN_D = 0

LANE_WIDTH = 4