std_msgs/Header header

# E-stop
bool estop_engaged

# Safety timer (velocity/effort timeout)
bool safety_timer_engaged      # true if no commands received within timeout
