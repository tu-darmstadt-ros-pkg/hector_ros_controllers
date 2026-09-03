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

# Collision diagnostics
float64 min_collision_distance       # current minimum collision distance [m]
uint32 num_pairs_in_safety_zone      # number of collision pairs in the safety zone

# Manipulability index (Yoshikawa w = sqrt(det(J*J^T)), 0 = singular)
float64 manipulability

# Safety QP diagnostics
bool qp_solved                        # last QP converged (v = QP solution)
bool qp_braking                       # fallback active: braking to zero at decel limit
bool qp_push_out_relaxed              # wedged: opposing contacts, push-out demands relaxed
                                      # to 'no-worsen'; escape motion still possible
uint32 qp_num_collision_constraints   # collision damper rows in the last QP
float64 qp_solve_time_us              # last QP solve time [us]
bool stalled                          # reference demands motion but commanded motion is ~0
                                      # (e.g. blocked head-on) -> upstream should replan
bool parked                           # stalled past stall_park_timeout: reference abandoned,
                                      # holding position until a NEW command arrives

# Active current limits per joint (only meaningful when current_limits_enabled is true)
string[] joint_names                 # joint names (same order as current_limits)
float64[] current_limits             # active current limit per joint [A] (compliant or stiff)
