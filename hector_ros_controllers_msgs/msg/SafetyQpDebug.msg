# Per-cycle introspection of the SafetyPositionController safety QP.
# Published on ~/qp_debug when the 'publish_qp_debug' parameter is enabled.
# Intended for PlotJuggler / debugging; not part of the stable interface.

std_msgs/Header header

# ---- Per-joint arrays (same order as the 'joints' parameter) ----
string[] joint_names
float64[] v_des        # desired velocity toward the (leashed) reference [rad/s]
float64[] v_cmd        # velocity chosen by the QP / fallback [rad/s]
float64[] box_lb       # per-joint velocity lower bound (vel+accel+position limits) [rad/s]
float64[] box_ub       # per-joint velocity upper bound [rad/s]
float64[] q_cmd        # configuration the collision check ran at [rad]
                       # position mode: the command written this cycle (integration state)
                       # velocity mode: the measured position plus one predicted step
float64[] q_ref        # reference position [rad]; NaN when the reference is a velocity
float64[] v_ref        # reference velocity [rad/s]; NaN when the reference is a position

# ---- Per collision-constraint arrays (closest first, as used in the QP) ----
string[] pair_names          # 'geomA<->geomB'
float64[] pair_distances     # signed distance [m] (negative = penetrating)
float64[] pair_rhs           # required minimum of g^T v (after any relaxation) [m/s]
float64[] pair_velocities    # achieved g^T v at the solution [m/s]
                             # binding constraint: pair_velocities ~= pair_rhs
                             # sinking despite pair_velocities >= pair_rhs -> linearization sag

# ---- Solver state ----
bool solved
bool braking
bool push_out_relaxed
bool bounds_conflict
float64 solve_time_us
int64 iterations
