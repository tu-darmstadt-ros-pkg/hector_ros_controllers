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
