#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <safety_position_controller/joint_info.hpp>
#include <safety_position_controller/safety_qp_limiter.hpp>
#include <safety_position_controller/stall_park_monitor.hpp>

namespace safety_position_controller
{

/**
 * @brief ROS-free per-cycle safety pipeline between the processed reference and the
 * commanded positions.
 *
 * Owns the desired velocity + reference leash, the position-limit, per-joint deviation
 * and tracking-leash boxes, collision damper constraint assembly, the QP solve
 * (SafetyQpLimiter), integration and the stall/park state machine.
 * Reports events instead of logging; the caller (controller) translates them.
 *
 * Per-cycle protocol (the collision check must run at the commanded configuration,
 * which lives here — hence two phases):
 *   1. prepare(reference, measured, bypass)  → rebase, v_des, boxes, park hold/resume
 *   2. caller evaluates collisions at commandedPositions()
 *   3. step(observation)                     → constraints, solve, integrate, stall/park
 *
 * Every bound is a box the QP solves against, so what step 3 writes is one bounded step
 * away from the configuration checked in step 2, taken under the velocity, acceleration
 * and collision-damper limits that check produced - not a correction applied afterwards
 * that none of them saw.
 *
 * Not thread-safe; call from the control thread only.
 */
class SafetyPipeline
{
public:
  struct Config {
    double dt{ 0.01 };
    std::vector<JointInfo> joints; ///< per controlled joint, defines n
    SafetyQpParams qp;             ///< v_max/a_acc/a_dec must be sized like joints
    /// Per-joint max deviation [rad] of the command from the leashed reference while
    /// flowing around collisions; 0 disables for that joint.
    std::vector<double> deviation_limits;
    /// Controlled joint index → collision-model velocity-space index (-1 if absent).
    /// May be empty when no collision model is used.
    std::vector<int> joint_v_index;
    double reference_leash_time{ 0.3 }; ///< bounds reference run-ahead; 0 disables
    /// Anti-windup box on |command - measured| [rad]; 0 disables. Must exceed the
    /// hardware's nominal following lag, or the box throttles the commanded speed.
    double tracking_leash{ 0.5 };
    double bypass_limit_tolerance{ 0.0 };    ///< position-limit extension (fraction of range)
    double stall_velocity_threshold{ 0.01 }; ///< |v| below this counts as not moving
    double park_resume_threshold{ 0.01 };    ///< reference change that counts as new command
    /// If true, a joint whose desired velocity stays below hold_velocity_threshold is
    /// pinned: the QP may not recruit it to flow around or push out of a collision, so
    /// a resting limb is never swept aside by a collision it did not cause.
    bool hold_unrequested{ false };
    double hold_velocity_threshold{ 0.01 }; ///< |v_des| below this counts as "not requested"
    StallParkMonitor::Params stall_park;
  };

  /// One collision pair candidate: signed distance + distance gradient dd/dv in the
  /// collision model's velocity space (projected onto the controlled joints via
  /// joint_v_index). The gradient pointer must stay valid until step() returns.
  struct PairCandidate {
    double distance{ 0.0 };
    const Eigen::VectorXd *gradient{ nullptr };
    std::size_t pair_index{ 0 };
  };

  /// How the collision state was observed this cycle.
  struct CollisionObservation {
    bool checks_active{ false }; ///< collision checking ran this cycle (not bypassed/disabled)
    bool state_valid{ true };    ///< joint state reads for the check succeeded; false → the
                                 ///< safety state is unobservable and the demand is zeroed
    bool in_collision{ false };
    const std::vector<PairCandidate> *pairs{ nullptr }; ///< safety-zone pairs (may be null)
  };

  struct Events {
    StallParkMonitor::Events stall;
  };

  /**
   * @param config see Config; joints must be non-empty and sizes consistent
   * @throws std::invalid_argument on inconsistent config (also from SafetyQpLimiter),
   * or when tracking_leash is too large to bound a continuous joint's wrapped lag
   */
  explicit SafetyPipeline( Config config );

  /// Rebase to the measured state and park on the next prepare() (E-stop, state-read
  /// failures): the reference in effect at that moment is abandoned, so nothing moves
  /// until a new one arrives.
  void invalidate()
  {
    state_valid_ = false;
    park_pending_ = true;
  }

  /**
   * @brief Phase 1: rebase if invalidated, clamp the reference to the position limits,
   * desired velocity toward the (leashed) reference, position-limit, deviation and
   * tracking-leash boxes, park hold/resume.
   * @param reference raw reference per joint (non-finite entries demand zero velocity);
   * for continuous joints the shortest path to the target is taken
   * @param measured measured positions per joint
   * @param bypass_active relaxes position limits and drops the deviation boxes
   * @return true if a new reference released the parked state this cycle
   */
  bool prepare( const std::vector<double> &reference, const std::vector<double> &measured,
                bool bypass_active );

  /// Current commanded configuration: evaluate the collision check here after prepare();
  /// after step() these are the positions to write to the hardware.
  const Eigen::VectorXd &commandedPositions() const { return cmd_; }

  /**
   * @brief Phase 2: assemble collision damper constraints, solve, integrate and update
   * the stall/park state machine.
   * @param obs collision observation for this cycle (see CollisionObservation)
   * @return edge events of this cycle; results via the getters below
   */
  Events step( const CollisionObservation &obs );

  // ---- Introspection (valid after step()) ----
  const SafetyQpResult &lastResult() const { return limiter_->lastResult(); }
  const SafetyQpInput &qpInput() const { return input_; }
  const Eigen::VectorXd &velocity() const { return vel_; }
  const Eigen::VectorXd &leashedReference() const { return ref_leashed_; }
  /// Collision-pair indices of the constraints in qpInput().collisions (same order).
  const std::vector<std::size_t> &constraintPairIndices() const { return constraint_pair_indices_; }
  const SafetyQpLimiter &limiter() const { return *limiter_; }
  bool stalled() const { return monitor_.stalled(); }
  bool parked() const { return monitor_.parked(); }
  double stallTime() const { return monitor_.stallTime(); }
  bool wantsMotion() const { return wants_motion_; }
  /// True when a joint is further from its command than the tracking leash can explain:
  /// the hardware left on its own, so the configuration the collision check runs at no
  /// longer describes the robot. Valid after prepare(); always false without a leash.
  bool measurementDiverged() const { return measurement_diverged_; }
  /// Per-joint hold flags of the current cycle (size n; all zero unless
  /// Config::hold_unrequested). Valid after prepare().
  const std::vector<uint8_t> &heldJoints() const { return input_.hold; }
  const Config &config() const { return config_; }

private:
  /// True if the reference differs from the one latched at park time (a new command).
  bool isNewReference( const std::vector<double> &reference ) const;

  /// Hold every joint (used where step() zeroes the demand after prepare() ran).
  void holdAll();

  Config config_;
  std::unique_ptr<SafetyQpLimiter> limiter_;
  StallParkMonitor monitor_;

  /// false → cmd_/vel_ are rebased to the measured state on the next prepare()
  bool state_valid_{ false };
  /// true → the next prepare() parks and latches its reference as abandoned. Separate
  /// from state_valid_: the rebase at construction/activation must not park, since the
  /// reference the upstream controller is holding there is a live target.
  bool park_pending_{ false };
  bool wants_motion_{ false };
  bool measurement_diverged_{ false };
  Eigen::VectorXd cmd_;         ///< commanded positions (integration state)
  Eigen::VectorXd vel_;         ///< commanded velocities
  Eigen::VectorXd ref_leashed_; ///< leashed reference targets (deviation is measured
                                ///< against these, so lag cannot blow the budget)
  SafetyQpInput input_;
  std::vector<double> reference_;        ///< this cycle's reference (park latch source)
  std::vector<double> parked_reference_; ///< reference snapshot latched at park time
  std::vector<std::size_t> constraint_pair_indices_;
};

} // namespace safety_position_controller
