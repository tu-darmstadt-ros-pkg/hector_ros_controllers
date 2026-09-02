#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace proxsuite::proxqp::dense
{
template<typename T>
struct QP;
} // namespace proxsuite::proxqp::dense

namespace safety_position_controller
{

/**
 * @brief Parameters for the velocity-damper safety QP.
 *
 * All vectors have size n (number of controlled joints).
 */
struct SafetyQpParams {
  double dt{ 0.01 };     ///< controller period [s]
  Eigen::VectorXd v_max; ///< per-joint velocity limits [rad/s]
  Eigen::VectorXd a_acc; ///< per-joint acceleration limits (increasing |v|) [rad/s^2]
  Eigen::VectorXd a_dec; ///< per-joint deceleration limits (decreasing |v|) [rad/s^2], >= a_acc

  // Collision velocity damper (Faverjon-Tournassoud): g^T v >= -xi * (d - d_pad)/(d_zone - d_pad)
  double damper_xi{ 2.0 }; ///< damper gain [1/s]: max approach speed at the outer zone edge
  double d_pad{ 0.01 };    ///< collision padding [m] (distance <= d_pad counts as collision)
  double d_zone{ 0.05 };   ///< outer safety zone [m] (damper inactive beyond this)
  double max_repulsion_speed{ 0.05 }; ///< cap on the damper RHS when d < d_pad (push-out) [m/s]

  /// Velocity clamp [rad/s] while any collision pair is inside the padding. Tangential
  /// sliding on curved geometry sinks the true distance ~quadratically with speed
  /// (linearization sag); crawling keeps the capped push-out stronger than the sag.
  /// The clamp decays from the current velocity at the deceleration limit (no conflict
  /// with the acceleration box). 0 disables.
  double contact_crawl_speed{ 0.0 };
  double regularization{ 1e-8 };               ///< added to the objective Hessian diagonal
  std::size_t max_collision_constraints{ 10 }; ///< preallocated collision rows (closest kept)
};

/// One linearized collision constraint: normal^T v >= damper(distance).
/// normal is dd/dv projected onto the controlled joints (size n).
struct QpCollisionConstraint {
  Eigen::VectorXd normal;
  double distance{ 0.0 };
};

/// Per-cycle input. All vectors size n.
struct SafetyQpInput {
  Eigen::VectorXd v_des;  ///< desired velocity toward the reference [rad/s]
  Eigen::VectorXd v_prev; ///< previously commanded velocity [rad/s]
  Eigen::VectorXd q;      ///< current commanded positions [rad] (for position limits)
  Eigen::VectorXd q_lo;   ///< lower position limits (-inf if unbounded)
  Eigen::VectorXd q_hi;   ///< upper position limits (+inf if unbounded)
  /// Per-joint hold flags (size n, or empty for "no joint is held"). A held joint is
  /// clamped toward zero velocity at its deceleration limit and then pinned there, so
  /// the QP cannot recruit it to flow around or push out of a collision.
  std::vector<uint8_t> hold;
  std::vector<QpCollisionConstraint> collisions; ///< sorted closest-first; extra rows dropped
};

/// Per-cycle output.
struct SafetyQpResult {
  Eigen::VectorXd v;             ///< commanded velocity [rad/s]
  bool solved{ false };          ///< QP converged; v is the QP solution
  bool braking{ false };         ///< fallback: v brakes to zero at the deceleration limit
  bool bounds_conflict{ false }; ///< per-joint bounds were mutually exclusive (resolved by braking)
  /// The full QP was infeasible (e.g. wedged between opposing contacts demanding
  /// incompatible push-outs); v comes from the relaxed "no-worsen" QP that only forbids
  /// getting deeper. Motion out of or along the contacts is still possible.
  bool push_out_relaxed{ false };
  int num_collision_constraints{ 0 };
  double solve_time_us{ 0.0 };
  long iterations{ 0 };
  // Per collision constraint (size num_collision_constraints), for debugging:
  Eigen::VectorXd collision_rhs;      ///< final required minimum of g^T v (after relaxation)
  Eigen::VectorXd collision_velocity; ///< achieved g^T v at the returned v
};

/**
 * @brief Small dense QP that projects a desired joint velocity onto the safe set.
 *
 * min_v  0.5 * ||v - v_des||^2  subject to
 *   - velocity limits            -v_max <= v <= v_max
 *   - acceleration limits        v_prev - a*dt <= v <= v_prev + a*dt (asymmetric acc/dec)
 *   - position limit braking     |v| <= sqrt(2 * a_dec * dist_to_limit)
 *   - collision velocity dampers g_k^T v >= -xi * (d_k - d_pad)/(d_zone - d_pad)
 *
 * Instead of stopping at obstacles, the solution slides tangentially along the active
 * constraints ("flow around"). If the QP is infeasible or the solver fails, the result
 * falls back to braking to zero at the deceleration limit.
 *
 * Not thread-safe; call solve() from a single (control) thread. The QP matrices are
 * allocated in the constructor; solve() still allocates its result vectors.
 */
class SafetyQpLimiter
{
public:
  /**
   * @param n number of controlled joints
   * @param params see SafetyQpParams; v_max/a_acc/a_dec must have size n
   * @throws std::invalid_argument on inconsistent sizes/values
   */
  SafetyQpLimiter( std::size_t n, SafetyQpParams params );
  ~SafetyQpLimiter();

  SafetyQpLimiter( const SafetyQpLimiter & ) = delete;
  SafetyQpLimiter &operator=( const SafetyQpLimiter & ) = delete;

  /**
   * @brief Solve the per-cycle QP.
   * @param input see SafetyQpInput; all vectors must have size n
   * @return velocity command + diagnostics (never NaN; falls back to braking). Refers
   * to a member workspace: valid until the next solve(), like lastBoxLower()/Upper().
   */
  const SafetyQpResult &solve( const SafetyQpInput &input );

  /**
   * @brief Desired velocity toward a target with overshoot guard.
   * Magnitude is min(|diff|/dt, v_max, sqrt(2 * a_dec * |diff|)) — the last term starts
   * braking early enough to stop AT the target given the deceleration limit.
   * @param diff signed distance to target [rad]
   * @param v_max velocity limit [rad/s]
   * @param a_dec deceleration limit [rad/s^2]
   * @param dt cycle time [s]
   * @return signed desired velocity [rad/s]
   */
  static double desiredVelocity( double diff, double v_max, double a_dec, double dt );

  const SafetyQpParams &params() const { return params_; }
  std::size_t size() const { return n_; }

  /// Result of the last solve(); same lifetime as the box bounds below.
  const SafetyQpResult &lastResult() const { return result_; }

  /// Per-joint velocity bounds used by the LAST solve (velocity + acceleration +
  /// position-limit braking, after conflict resolution). For debugging/introspection.
  const Eigen::VectorXd &lastBoxLower() const { return box_lb_; }
  const Eigen::VectorXd &lastBoxUpper() const { return box_ub_; }

  /// Update damper distances (e.g. after a parameter change). Gains/limits stay fixed.
  void setCollisionDistances( double d_pad, double d_zone );

private:
  /// Per-joint velocity box from velocity/acceleration/position limits.
  /// On a per-joint conflict (cannot satisfy both), the box collapses to the braking
  /// velocity and conflict is set.
  /// @param crawl if true, additionally clamp toward +-contact_crawl_speed (decaying
  /// from the previous velocity at the deceleration limit). Joints flagged in
  /// input.hold get the same clamp toward zero, which overrides the crawl speed.
  void computeVelocityBounds( const SafetyQpInput &input, bool crawl, bool &conflict );

  /// Velocity that brakes toward zero at the deceleration limit, written into @p v.
  void brakingVelocity( const Eigen::VectorXd &v_prev, Eigen::VectorXd &v ) const;

  std::size_t n_;
  SafetyQpParams params_;

  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  bool first_solve_{ true };

  // Preallocated QP matrices: rows = n (box) + max_collision_constraints + 1 (deviation)
  Eigen::MatrixXd H_, C_;
  Eigen::VectorXd g_, l_, u_;
  Eigen::VectorXd box_lb_, box_ub_; // per-joint bounds (also used to clamp the solution)
  SafetyQpResult result_;           // reused across solves; returned by reference
};

} // namespace safety_position_controller
