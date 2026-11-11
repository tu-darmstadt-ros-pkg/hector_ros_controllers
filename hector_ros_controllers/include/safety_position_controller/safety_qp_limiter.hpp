#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace safety_position_controller
{

class DualActiveSetQp;

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

/// Linearized collision constraints normal(k)^T v >= damper(distance(k)), k < size().
/// A normal is dd/dv projected onto the controlled joints. The storage is sized once by
/// reserve(), so refilling it every cycle does not allocate.
class QpCollisionConstraints
{
public:
  /// Room for @p capacity constraints over @p n joints; drops the stored ones.
  void reserve( Eigen::Index capacity, Eigen::Index n );

  void clear() { size_ = 0; }

  /// Append a constraint.
  /// @return false (nothing stored) if the storage is full
  /// @throws std::invalid_argument if @p normal does not have size n
  bool push( const Eigen::Ref<const Eigen::VectorXd> &normal, double distance );

  Eigen::Index size() const { return size_; }
  bool empty() const { return size_ == 0; }
  Eigen::Index joints() const { return normals_.cols(); }
  auto normal( const Eigen::Index k ) const { return normals_.row( k ); }
  double distance( const Eigen::Index k ) const { return distances_[k]; }
  /// The normals of the stored constraints, one per row.
  auto normals() const { return normals_.topRows( size_ ); }

private:
  Eigen::MatrixXd normals_;
  Eigen::VectorXd distances_;
  Eigen::Index size_{ 0 };
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
  QpCollisionConstraints collisions; ///< sorted closest-first; extra rows dropped
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
  long iterations{ 0 }; ///< active-set additions and drops, summed over all solve stages
  // Per collision constraint, for debugging. Sized max_collision_constraints; the first
  // num_collision_constraints entries are valid.
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
 * falls back to braking to zero at the deceleration limit. The QP is solved exactly by a
 * dual active-set method (Goldfarb-Idnani).
 *
 * Not thread-safe; call solve() from a single (control) thread. All storage is allocated
 * in the constructor; solve() does not allocate.
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
   * @throws std::invalid_argument if the collision normals do not have size n
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

  std::unique_ptr<DualActiveSetQp> qp_;

  // Preallocated QP matrices: rows = n (box) + max_collision_constraints
  Eigen::MatrixXd C_;
  Eigen::VectorXd l_, u_;
  Eigen::VectorXd box_lb_, box_ub_; // per-joint bounds (also used to clamp the solution)
  SafetyQpResult result_;           // reused across solves; returned by reference
};

} // namespace safety_position_controller
