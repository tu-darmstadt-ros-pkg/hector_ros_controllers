std_msgs/Header header

# Safety bypass (collision check + relaxed joint limits)
bool safety_bypass_active

# Current limit mode (only meaningful when current_limits_enabled is true)
bool compliant_mode            # true = soft/compliant limits, false = stiff limits
bool current_limits_enabled    # true if set_current_limits param is on

# Self-collision checking
bool collision_check_enabled   # true if check_self_collisions param is on

# E-stop
bool estop_engaged

# Position limit enforcement
bool position_limits_enforced  # true if enforce_position_limits param is on

# Directional collision scaling diagnostics
float64 min_collision_distance       # current minimum collision distance [m]
float64 distance_scale               # raw distance-based velocity scale [0,1]
float64 effective_scale              # after directional override [0,1]
float64 worst_directional_derivative # min dot product across safety-zone pairs (positive = away, NaN = not evaluated)
uint32 num_pairs_in_safety_zone      # number of collision pairs in the safety zone

# Manipulability index (Yoshikawa w = sqrt(det(J*J^T)), 0 = singular)
float64 manipulability

# Active current limits per joint (only meaningful when current_limits_enabled is true)
string[] joint_names                 # joint names (same order as current_limits)
float64[] current_limits             # active current limit per joint [A] (compliant or stiff)
