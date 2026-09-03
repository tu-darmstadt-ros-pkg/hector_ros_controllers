// Pure unit tests for SafetyPipeline: no ROS runtime, no controller fixture.
// Covers the per-cycle protocol (prepare → collision observation → step), the
// stall/park state machine and the collision constraint assembly.

#include <gmock/gmock.h>

#include <safety_position_controller/safety_pipeline.hpp>

namespace spc = safety_position_controller;
using Pipeline = spc::SafetyPipeline;

namespace
{
constexpr double kDt = 0.01;
constexpr double kVMax = 1.0;
constexpr double kAcc = 8.0;
constexpr double kDec = 24.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Pipeline::Config makeConfig( const size_t n = 2 )
{
  Pipeline::Config cfg;
  cfg.dt = kDt;
  spc::JointInfo joint;
  joint.type = spc::JointType::REVOLUTE_BOUNDED;
  joint.has_position_limits = true;
  joint.lower_limit = -3.0;
  joint.upper_limit = 3.0;
  joint.velocity_limit = kVMax;
  cfg.joints.assign( n, joint );
  const auto ni = static_cast<Eigen::Index>( n );
  cfg.qp.dt = kDt;
  cfg.qp.v_max = Eigen::VectorXd::Constant( ni, kVMax );
  cfg.qp.a_acc = Eigen::VectorXd::Constant( ni, kAcc );
  cfg.qp.a_dec = Eigen::VectorXd::Constant( ni, kDec );
  cfg.qp.d_pad = 0.0;
  cfg.qp.d_zone = 0.05;
  // Contact at exactly d_pad must be a hard block, not a capped push-out: the park
  // tests need a true stall.
  cfg.qp.max_repulsion_speed = 0.0;
  // At a degenerate active boundary (damper rhs exactly 0 with v_des pulling against
  // it) proxqp reports solved with O(0.01) slack in v (see the 0.05 stall allowance in
  // the SafetyQpLimiter head-on test). The stall threshold must sit above that slop.
  cfg.stall_velocity_threshold = 0.05;
  cfg.deviation_limits.assign( n, 0.0 );
  cfg.joint_v_index.resize( n );
  for ( size_t i = 0; i < n; ++i ) { cfg.joint_v_index[i] = static_cast<int>( i ); }
  cfg.stall_park.stall_timeout = 0.2;
  cfg.stall_park.park_timeout = 0.5;
  return cfg;
}

/// One cycle; the hardware follows the command perfectly.
Pipeline::Events cycle( Pipeline &pipeline, const std::vector<double> &reference,
                        std::vector<double> &measured,
                        const Pipeline::CollisionObservation &obs = {}, const bool bypass = false )
{
  pipeline.prepare( reference, measured, bypass );
  const auto events = pipeline.step( obs );
  for ( size_t i = 0; i < measured.size(); ++i ) {
    measured[i] = pipeline.commandedPositions()[static_cast<Eigen::Index>( i )];
  }
  return events;
}
} // namespace

TEST( SafetyPipeline, TracksReferenceWithBoundedVelocityAndAcceleration )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  const std::vector<double> reference{ 0.5, 0.0 };

  double prev_cmd = 0.0, prev_v = 0.0, max_dv = 0.0, max_v = 0.0;
  for ( int i = 0; i < 300; ++i ) {
    cycle( pipeline, reference, measured );
    const double cmd = measured[0];
    const double v = ( cmd - prev_cmd ) / kDt;
    max_dv = std::max( max_dv, std::abs( v - prev_v ) );
    max_v = std::max( max_v, std::abs( v ) );
    prev_cmd = cmd;
    prev_v = v;
  }
  EXPECT_NEAR( measured[0], 0.5, 1e-4 );
  EXPECT_LE( max_v, kVMax + 1e-6 );
  EXPECT_LE( max_dv, kDec * kDt + 1e-6 );
}

TEST( SafetyPipeline, NaNReferenceHoldsPosition )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.4, -0.2 };
  const std::vector<double> reference{ kNaN, kNaN };

  for ( int i = 0; i < 20; ++i ) { cycle( pipeline, reference, measured ); }
  EXPECT_NEAR( measured[0], 0.4, 1e-6 );
  EXPECT_NEAR( measured[1], -0.2, 1e-6 );
  EXPECT_FALSE( pipeline.wantsMotion() );
}

TEST( SafetyPipeline, NonFiniteReferenceIsTreatedAsNoTarget )
{
  // A continuous joint has no position limits to clamp an infinity against, so it used
  // to reach get_signed_distance() and turn the whole desired velocity into NaN, which
  // the solver could only answer by failing over to braking.
  auto cfg = makeConfig( 1 );
  cfg.joints[0].type = spc::JointType::CONTINUOUS;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.5 };
  for ( int i = 0; i < 20; ++i ) {
    cycle( pipeline, { std::numeric_limits<double>::infinity() }, measured );
    ASSERT_TRUE( pipeline.lastResult().solved ) << "the QP must never be fed a NaN demand";
  }
  EXPECT_NEAR( measured[0], 0.5, 1e-6 );
  EXPECT_FALSE( pipeline.wantsMotion() );
}

TEST( SafetyPipeline, InvalidateRebasesAndParksUntilNewReference )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  for ( int i = 0; i < 50; ++i ) { cycle( pipeline, { 0.5, 0.0 }, measured ); }
  EXPECT_GT( measured[0], 0.1 );

  // E-stop style: state invalidated, robot ends up somewhere else
  pipeline.invalidate();
  measured = { 1.0, -0.5 };
  cycle( pipeline, { 0.5, 0.0 }, measured );
  EXPECT_NEAR( measured[0], 1.0, 1e-9 );
  EXPECT_NEAR( measured[1], -0.5, 1e-9 );
  EXPECT_NEAR( pipeline.velocity().cwiseAbs().maxCoeff(), 0.0, 1e-9 );
  EXPECT_TRUE( pipeline.parked() );

  // The reference that was in effect at the invalidation is abandoned: the unchanged
  // reference must not pull the arm back, no matter how long it keeps being commanded.
  for ( int i = 0; i < 50; ++i ) { cycle( pipeline, { 0.5, 0.0 }, measured ); }
  EXPECT_NEAR( measured[0], 1.0, 1e-9 );

  // A changed reference is a new command and releases the park.
  for ( int i = 0; i < 80; ++i ) { cycle( pipeline, { 0.6, 0.0 }, measured ); }
  EXPECT_FALSE( pipeline.parked() );
  EXPECT_NEAR( measured[0], 0.6, 1e-3 );
}

TEST( SafetyPipeline, NonFiniteReferenceEntryDoesNotReleasePark )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  pipeline.invalidate();
  cycle( pipeline, { 0.5, 0.0 }, measured );
  ASSERT_TRUE( pipeline.parked() );

  // An inf glitch means "no target" (as in prepare()), not a new command: it must not
  // release the park and resume the abandoned reference on the other joints.
  for ( int i = 0; i < 20; ++i ) {
    cycle( pipeline, { std::numeric_limits<double>::infinity(), 0.0 }, measured );
  }
  EXPECT_TRUE( pipeline.parked() );
  EXPECT_NEAR( measured[0], 0.0, 1e-9 );

  // A genuinely different finite reference still releases it.
  cycle( pipeline, { 0.2, 0.0 }, measured );
  EXPECT_FALSE( pipeline.parked() );
}

TEST( SafetyPipeline, TrackingLeashFollowsContinuousJointsAcrossTheWrap )
{
  // The hardware may report wrapped angles while cmd_ integrates freely. At the wrap the
  // raw difference is ~2*pi, which must not be mistaken for tracking lag: rebasing onto
  // the measured frame there would step the command by almost a full turn in one cycle,
  // past every velocity and acceleration bound.
  auto cfg = makeConfig( 1 );
  cfg.joints[0].type = spc::JointType::CONTINUOUS;
  cfg.tracking_leash = 0.5;
  Pipeline pipeline( cfg );

  const auto wrap = []( const double angle ) { return spc::get_signed_distance( 0.0, angle ); };

  std::vector<double> measured{ 3.0 }; // just below +pi
  double previous = 3.0;
  for ( int i = 0; i < 100; ++i ) {
    const double reference = wrap( pipeline.commandedPositions()[0] + 0.3 );
    pipeline.prepare( { reference }, measured, false );
    pipeline.step( {} );
    const double cmd = pipeline.commandedPositions()[0];
    ASSERT_LE( std::abs( cmd - previous ), kVMax * kDt + 1e-9 ) << "command jumped in cycle " << i;
    previous = cmd;
    measured[0] = wrap( cmd ); // hardware follows, reporting in [-pi, pi]
  }
  EXPECT_GT( pipeline.commandedPositions()[0], M_PI ) << "should have rotated past the wrap";
}

TEST( SafetyPipeline, TrackingLeashStillBoundsContinuousJointLag )
{
  auto cfg = makeConfig( 1 );
  cfg.joints[0].type = spc::JointType::CONTINUOUS;
  cfg.tracking_leash = 0.2;
  Pipeline pipeline( cfg );

  // Hardware stuck at 0 while the reference pulls away: the command must not wind up.
  std::vector<double> measured{ 0.0 };
  for ( int i = 0; i < 100; ++i ) {
    pipeline.prepare( { 1.5 }, measured, false );
    pipeline.step( {} );
    ASSERT_LE( pipeline.commandedPositions()[0], 0.2 + 1e-3 )
        << "the leash must bound every cycle, not just the settled state; cycle " << i;
  }
  EXPECT_NEAR( pipeline.commandedPositions()[0], 0.2, 1e-3 );
  EXPECT_TRUE( pipeline.stalled() ) << "hardware that cannot follow is a stall";
}

TEST( SafetyPipeline, TrackingLeashRespectsVelocityAndAccelerationBoxes )
{
  // The leash is a bound the QP solves against, so the command it produces is the one
  // that was collision-checked and the reported velocity is the one actually applied.
  // Correcting the command after the solve breaks both: it moves the command outside
  // the QP's velocity/acceleration boxes while the solver still reports full speed.
  auto cfg = makeConfig( 1 );
  cfg.tracking_leash = 0.2;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 }; // hardware stuck
  double previous_cmd = 0.0;
  double previous_vel = 0.0;
  for ( int i = 0; i < 100; ++i ) {
    pipeline.prepare( { 1.5 }, measured, false );
    pipeline.step( {} );
    const double cmd = pipeline.commandedPositions()[0];
    const double vel = pipeline.velocity()[0];
    ASSERT_NEAR( cmd - previous_cmd, vel * kDt, 1e-9 )
        << "the command must move by exactly the reported velocity in cycle " << i;
    ASSERT_LE( std::abs( cmd - previous_cmd ), kVMax * kDt + 1e-9 ) << "cycle " << i;
    ASSERT_LE( std::abs( vel - previous_vel ), kDec * kDt + 1e-9 ) << "cycle " << i;
    previous_cmd = cmd;
    previous_vel = vel;
  }
  EXPECT_NEAR( pipeline.commandedPositions()[0], 0.2, 1e-3 );
  EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-3 ) << "settled against the leash";
}

TEST( SafetyPipeline, TrackingLeashHoldsWhenTheMeasurementJumpsAway )
{
  // A measurement that runs away from the command (a slipping or backdriven joint, a
  // re-homed encoder) must not drag the command after it: the held command is the one
  // that was collision-checked, a dragged one never was.
  auto cfg = makeConfig( 1 );
  cfg.tracking_leash = 0.2;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 };
  cycle( pipeline, { 0.0 }, measured );
  ASSERT_NEAR( pipeline.commandedPositions()[0], 0.0, 1e-9 );

  measured[0] = -1.0; // the joint left on its own
  for ( int i = 0; i < 20; ++i ) {
    pipeline.prepare( { 0.0 }, measured, false );
    pipeline.step( {} );
    ASSERT_NEAR( pipeline.commandedPositions()[0], 0.0, 1e-9 )
        << "the command must hold, not chase the measurement, in cycle " << i;
  }
}

TEST( SafetyPipeline, MeasurementRunningAwayIsReportedAsDiverged )
{
  // The box bounds how far the command may LEAD the joint and deliberately does not
  // chase a joint that leaves on its own, so something has to notice when it does: the
  // collision check runs at the command, which then no longer describes the robot.
  auto cfg = makeConfig( 1 );
  cfg.tracking_leash = 0.2;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 };
  cycle( pipeline, { 0.0 }, measured );
  EXPECT_FALSE( pipeline.measurementDiverged() );

  // Pinned against the leash by a blockage is NOT divergence: the flipper case must not
  // trip the watchdog, however long it pushes.
  for ( int i = 0; i < 200; ++i ) {
    measured[0] = 0.0;
    pipeline.prepare( { 1.5 }, measured, false );
    pipeline.step( {} );
    ASSERT_FALSE( pipeline.measurementDiverged() ) << "leash-pinned is not diverged, cycle " << i;
  }

  // The joint leaving on its own is.
  measured[0] = -0.5; // 0.7 from the command, past 2x the leash
  pipeline.prepare( { 1.5 }, measured, false );
  pipeline.step( {} );
  EXPECT_TRUE( pipeline.measurementDiverged() );

  // And it clears once the joint is back within reach.
  measured[0] = pipeline.commandedPositions()[0] - 0.1;
  pipeline.prepare( { 1.5 }, measured, false );
  pipeline.step( {} );
  EXPECT_FALSE( pipeline.measurementDiverged() );
}

TEST( SafetyPipeline, ContinuousJointDivergenceIsCaughtBeforeTheWrapAliases )
{
  // A continuous joint's lag is only known modulo a full turn. Once the true lag passes
  // pi the wrapped value names the other side of the joint, which would mirror the box
  // onto the wrong side of the command and lock the joint one way. The divergence bound
  // must be reached first, while the wrapped lag is still faithful.
  auto cfg = makeConfig( 1 );
  cfg.joints[0].type = spc::JointType::CONTINUOUS;
  cfg.joints[0].has_position_limits = false;
  cfg.tracking_leash = 0.5;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 };
  cycle( pipeline, { 0.0 }, measured );

  // The hardware is dragged backwards while the operator holds a negative command.
  double true_position = measured[0];
  bool diverged_reported = false;
  for ( int i = 0; i < 200; ++i ) {
    true_position -= 6.0 * kDt;                                   // dragged at 6 rad/s
    measured[0] = spc::get_signed_distance( 0.0, true_position ); // hardware wraps
    pipeline.prepare( { -1.0 }, measured, false );
    pipeline.step( {} );
    const double true_lag = pipeline.commandedPositions()[0] - true_position;
    if ( pipeline.measurementDiverged() ) {
      diverged_reported = true;
      EXPECT_LT( std::abs( true_lag ), M_PI )
          << "divergence must be reported before the wrapped lag can alias";
      break;
    }
    ASSERT_LT( std::abs( true_lag ), M_PI ) << "aliased before reporting, cycle " << i;
  }
  EXPECT_TRUE( diverged_reported ) << "a runaway continuous joint must be reported";
}

TEST( SafetyPipeline, ContinuousJointRefusesALeashItCannotBound )
{
  // Beyond this the divergence bound would sit past pi, where a wrapped lag can no
  // longer be told from its alias, so the wrap handling would have no guarantee left.
  auto cfg = makeConfig( 1 );
  cfg.joints[0].type = spc::JointType::CONTINUOUS;
  cfg.tracking_leash = M_PI / 2.0 + 0.01;
  EXPECT_THROW( Pipeline{ cfg }, std::invalid_argument );

  cfg.tracking_leash = M_PI / 2.0 - 0.01;
  EXPECT_NO_THROW( Pipeline{ cfg } );

  // A bounded joint has no wrap, so the same leash is fine there.
  auto bounded = makeConfig( 1 );
  bounded.tracking_leash = 3.0;
  EXPECT_NO_THROW( Pipeline{ bounded } );
}

TEST( SafetyPipeline, BlockedJointKeepsPushingAtTheLeash )
{
  // An upstream controller that integrates its own reference (the flipper velocity to
  // position controller) runs the reference to the joint limit while the limb is
  // blocked. The command must keep pushing at the leash rather than backing off: the
  // flipper carries the robot.
  auto cfg = makeConfig( 1 );
  cfg.tracking_leash = 0.2;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 }; // blocked by the ground
  double reference = 0.0;
  for ( int i = 0; i < 300; ++i ) {
    reference += kVMax * kDt; // upstream integrates on, unaware of the blockage
    pipeline.prepare( { reference }, measured, false );
    pipeline.step( {} );
  }
  EXPECT_NEAR( pipeline.commandedPositions()[0], 0.2, 1e-3 )
      << "still pushing at the leash, not backed off";
  EXPECT_TRUE( pipeline.stalled() );

  // The bound is anchored to the measurement, not to the command, so settling against
  // it cannot ratchet the command outward however long the blockage lasts.
  const double settled = pipeline.commandedPositions()[0];
  for ( int i = 0; i < 30000; ++i ) { // ten minutes at 50 Hz
    reference += kVMax * kDt;
    pipeline.prepare( { reference }, measured, false );
    pipeline.step( {} );
  }
  EXPECT_NEAR( pipeline.commandedPositions()[0], settled, 1e-9 ) << "the leash must not creep";
}

TEST( SafetyPipeline, ReferenceOutsideThePositionLimitsHoldsWithoutStalling )
{
  // A joint resting beyond its limit (e.g. after a bypass expired at a fold position)
  // must simply be held. The velocity box allows no outward motion there, so a demand
  // toward the outside would never be met and would be reported as a stall, then park.
  auto cfg = makeConfig( 1 );
  cfg.joints[0].lower_limit = -1.0;
  cfg.joints[0].upper_limit = 1.0;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 1.15 };
  pipeline.prepare( { 1.15 }, measured, false ); // rebase the command outside the limit
  pipeline.step( {} );
  measured[0] = 1.2; // hardware sits slightly further out than the command

  for ( int i = 0; i < 200; ++i ) {
    pipeline.prepare( { 1.5 }, measured, false );
    pipeline.step( {} );
  }
  EXPECT_NEAR( pipeline.commandedPositions()[0], 1.15, 1e-6 );
  EXPECT_FALSE( pipeline.stalled() ) << "holding at a limit is not a blocked path";
  EXPECT_FALSE( pipeline.parked() );

  // A reference back inside the limits is still followed.
  for ( int i = 0; i < 200; ++i ) {
    pipeline.prepare( { 0.5 }, measured, false );
    pipeline.step( {} );
    measured[0] = pipeline.commandedPositions()[0];
  }
  EXPECT_NEAR( pipeline.commandedPositions()[0], 0.5, 1e-3 );
}

TEST( SafetyPipeline, DeviationBoxAroundLeashedReferenceDroppedDuringBypass )
{
  auto cfg = makeConfig();
  cfg.deviation_limits = { 0.1, 0.25 };
  Pipeline pipeline( cfg );
  std::vector<double> measured{ 0.0, 0.0 };
  // leash: v_max(1.0) * 0.3 s → leashed reference 0.3
  const std::vector<double> reference{ 0.5, 0.0 };

  pipeline.prepare( reference, measured, false );
  pipeline.step( {} );
  // joint0 (limit 0.1): box = [min(0.3-0.1, cmd~0), 0.3+0.1] = [0.0, 0.4]
  EXPECT_NEAR( pipeline.qpInput().q_hi[0], 0.4, 1e-6 );
  EXPECT_NEAR( pipeline.qpInput().q_lo[0], 0.0, 1e-6 );
  // joint1 (limit 0.25, ref 0): box = [-0.25, 0.25] within URDF [-3, 3]
  EXPECT_NEAR( pipeline.qpInput().q_hi[1], 0.25, 1e-6 );
  EXPECT_NEAR( pipeline.qpInput().q_lo[1], -0.25, 1e-6 );

  // Bypass drops the deviation boxes. The tracking leash is anti-windup for the
  // hardware, not a collision constraint, so it survives the bypass exactly as the
  // position limits do.
  pipeline.prepare( reference, measured, true );
  EXPECT_NEAR( pipeline.qpInput().q_hi[0], 0.5, 1e-9 );
  EXPECT_NEAR( pipeline.qpInput().q_lo[0], -0.5, 1e-9 );

  auto no_leash = cfg;
  no_leash.tracking_leash = 0.0;
  Pipeline unleashed( no_leash );
  unleashed.prepare( reference, measured, true );
  EXPECT_NEAR( unleashed.qpInput().q_hi[0], 3.0, 1e-9 ) << "only the position limits remain";
  EXPECT_NEAR( unleashed.qpInput().q_lo[0], -3.0, 1e-9 );
}

TEST( SafetyPipeline, HeadOnBlockStallsParksAndResumesOnNewReference )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  std::vector<double> reference{ 2.0, 0.0 };

  // Contact at the padding, gradient opposing +joint0 motion → damper enforces v0 <= 0
  Eigen::VectorXd gradient( 2 );
  gradient << -1.0, 0.0;
  std::vector<Pipeline::PairCandidate> pairs{ { 0.0, &gradient, 3 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  int stalled_at = -1, parked_at = -1;
  for ( int i = 0; i < 100; ++i ) {
    const auto events = cycle( pipeline, reference, measured, obs );
    if ( events.stall.stalled && stalled_at < 0 ) {
      stalled_at = i;
    }
    if ( events.stall.parked && parked_at < 0 ) {
      parked_at = i;
    }
  }
  EXPECT_LT( std::abs( measured[0] ), 0.05 ) << "head-on block must stay at the boundary";
  EXPECT_EQ( stalled_at, 19 ) << "stall_timeout = 0.2 s = 20 cycles";
  EXPECT_EQ( parked_at, 49 ) << "park_timeout = 0.5 s = 50 cycles";
  ASSERT_TRUE( pipeline.parked() );
  EXPECT_EQ( pipeline.constraintPairIndices().front(), 3u );

  // While parked, the (unchanged) reference demands nothing — even if the blockage clears
  const double parked_cmd = measured[0];
  for ( int i = 0; i < 20; ++i ) { cycle( pipeline, reference, measured ); }
  EXPECT_TRUE( pipeline.parked() );
  EXPECT_FALSE( pipeline.wantsMotion() );
  EXPECT_NEAR( measured[0], parked_cmd, 1e-6 ) << "parked limb must not creep";

  // A NEW reference (retreat, allowed by the constraint) releases the park
  reference[0] = -0.5;
  pipeline.prepare( reference, measured, false );
  EXPECT_FALSE( pipeline.parked() );
  pipeline.step( obs );
  // Tolerance matches the boundary-constraint solver slop documented in makeConfig().
  for ( int i = 0; i < 300; ++i ) { cycle( pipeline, reference, measured, obs ); }
  EXPECT_NEAR( measured[0], -0.5, 0.05 );
}

TEST( SafetyPipeline, ResumeFromParkIsReportedByPrepare )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  Eigen::VectorXd gradient( 2 );
  gradient << -1.0, 0.0;
  std::vector<Pipeline::PairCandidate> pairs{ { 0.0, &gradient, 0 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  for ( int i = 0; i < 60; ++i ) { cycle( pipeline, { 2.0, 0.0 }, measured, obs ); }
  ASSERT_TRUE( pipeline.parked() );

  EXPECT_FALSE( pipeline.prepare( { 2.0, 0.0 }, measured, false ) )
      << "same reference stays parked";
  pipeline.step( obs );
  EXPECT_TRUE( pipeline.prepare( { 2.0, 0.3 }, measured, false ) )
      << "reference change on any joint releases the park";
  pipeline.step( obs );
  EXPECT_FALSE( pipeline.parked() );
}

TEST( SafetyPipeline, ParkSurvivesInvalidation )
{
  // A stale reference must stay abandoned across an E-stop (invalidate/rebase).
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  Eigen::VectorXd gradient( 2 );
  gradient << -1.0, 0.0;
  std::vector<Pipeline::PairCandidate> pairs{ { 0.0, &gradient, 0 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  for ( int i = 0; i < 60; ++i ) { cycle( pipeline, { 2.0, 0.0 }, measured, obs ); }
  ASSERT_TRUE( pipeline.parked() );

  pipeline.invalidate();
  const double rebased_cmd = measured[0];
  for ( int i = 0; i < 20; ++i ) { cycle( pipeline, { 2.0, 0.0 }, measured ); }
  EXPECT_TRUE( pipeline.parked() );
  EXPECT_FALSE( pipeline.stalled() ) << "stall accumulation resets on rebase";
  EXPECT_NEAR( measured[0], rebased_cmd, 1e-6 );
}

TEST( SafetyPipeline, UninfluenceablePairsAreDroppedAndCollisionWithoutConstraintsZeroesDemand )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };

  Eigen::VectorXd zero_gradient = Eigen::VectorXd::Zero( 2 );
  std::vector<Pipeline::PairCandidate> pairs{ { -0.01, &zero_gradient, 5 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.in_collision = true;
  obs.pairs = &pairs;

  for ( int i = 0; i < 10; ++i ) { cycle( pipeline, { 2.0, 0.0 }, measured, obs ); }
  EXPECT_TRUE( pipeline.qpInput().collisions.empty() )
      << "zero-projection pairs must not constrain the QP";
  EXPECT_FALSE( pipeline.wantsMotion() ) << "in collision without usable constraints → hold";
  EXPECT_NEAR( measured[0], 0.0, 1e-6 );
}

TEST( SafetyPipeline, UnobservableSafetyStateBrakesToAStop )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  const std::vector<double> reference{ 2.0, 0.0 };

  for ( int i = 0; i < 30; ++i ) { cycle( pipeline, reference, measured ); }
  ASSERT_NEAR( pipeline.velocity()[0], kVMax, 1e-6 );

  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = false; // joint state reads failed
  double prev_v = pipeline.velocity()[0];
  for ( int i = 0; i < 20; ++i ) {
    cycle( pipeline, reference, measured, obs );
    const double v = pipeline.velocity()[0];
    EXPECT_LE( v, prev_v + 1e-9 );
    EXPECT_GE( prev_v - v, -1e-9 );
    EXPECT_LE( prev_v - v, kDec * kDt + 1e-9 );
    prev_v = v;
  }
  EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-6 );
  EXPECT_FALSE( pipeline.wantsMotion() );
}

TEST( SafetyPipeline, ConstructorValidatesConfig )
{
  auto cfg = makeConfig();
  cfg.joints.clear();
  EXPECT_THROW( Pipeline{ cfg }, std::invalid_argument );

  cfg = makeConfig();
  cfg.deviation_limits.resize( 1 );
  EXPECT_THROW( Pipeline{ cfg }, std::invalid_argument );

  cfg = makeConfig();
  cfg.joint_v_index.resize( 1 );
  EXPECT_THROW( Pipeline{ cfg }, std::invalid_argument );

  cfg = makeConfig();
  cfg.joint_v_index.clear(); // allowed: no collision model
  EXPECT_NO_THROW( Pipeline{ cfg } );
}

// ---------------------------------------------------------------------------
// hold_unrequested: a joint the reference is not moving stays put
// ---------------------------------------------------------------------------

namespace
{
/// Joint 0 is driven toward +1 while joint 1 rests at its reference. The pair
/// separates when joint 0 goes negative OR joint 1 goes positive, so the QP's cheapest
/// escape is to sweep the resting joint aside — exactly the flipper case.
Eigen::VectorXd sweepGradient()
{
  Eigen::VectorXd g( 2 );
  g << -1.0, 1.0;
  return g;
}

/// Runs `cycles` steps against a pair held at `distance` and returns the measured state.
std::vector<double> runAgainstPair( Pipeline &pipeline, const double distance, const int cycles )
{
  const Eigen::VectorXd gradient = sweepGradient();
  std::vector<Pipeline::PairCandidate> pairs{ { distance, &gradient, 0 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  std::vector<double> measured{ 0.0, 0.0 };
  for ( int i = 0; i < cycles; ++i ) { cycle( pipeline, { 1.0, 0.0 }, measured, obs ); }
  return measured;
}
} // namespace

TEST( SafetyPipeline, WithoutHoldTheRestingJointIsSweptOutOfTheWay )
{
  // Documents the default behavior the hold mode exists to switch off.
  Pipeline pipeline( makeConfig() );
  const auto measured = runAgainstPair( pipeline, 0.001, 50 );

  EXPECT_GT( measured[1], 0.01 ) << "the resting joint gives way to let joint 0 through";
  EXPECT_GT( measured[0], 0.01 );
}

TEST( SafetyPipeline, HoldPinsTheRestingJointAndBlocksTheDrivenOne )
{
  auto cfg = makeConfig();
  cfg.hold_unrequested = true;
  Pipeline pipeline( cfg );
  const auto measured = runAgainstPair( pipeline, 0.001, 50 );

  EXPECT_EQ( measured[1], 0.0 ) << "a joint the reference does not move must not move";
  // Joint 0 may still creep along whatever slack the damper leaves, but it can no
  // longer buy room by pushing joint 1 away.
  Pipeline reference_run( makeConfig() );
  const auto unheld = runAgainstPair( reference_run, 0.001, 50 );
  EXPECT_LT( measured[0], unheld[0] );
}

TEST( SafetyPipeline, HoldLeavesTheCommandedJointFree )
{
  // The mode must only pin joints that are NOT being commanded.
  auto cfg = makeConfig();
  cfg.hold_unrequested = true;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0, 0.0 };
  for ( int i = 0; i < 50; ++i ) { cycle( pipeline, { 1.0, 1.0 }, measured ); }

  EXPECT_GT( measured[0], 0.1 );
  EXPECT_GT( measured[1], 0.1 );
  EXPECT_THAT( pipeline.heldJoints(), ::testing::ElementsAre( 0, 0 ) );
}

TEST( SafetyPipeline, HoldFlagsFollowTheDesiredVelocity )
{
  auto cfg = makeConfig();
  cfg.hold_unrequested = true;
  cfg.hold_velocity_threshold = 0.01;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0, 0.0 };
  // Joint 0 far from its target, joint 1 already there.
  pipeline.prepare( { 1.0, 0.0 }, measured, false );
  EXPECT_THAT( pipeline.heldJoints(), ::testing::ElementsAre( 0, 1 ) );

  // A NaN reference means "no target", which is not a request to move either.
  pipeline.prepare( { kNaN, 0.0 }, measured, false );
  EXPECT_THAT( pipeline.heldJoints(), ::testing::ElementsAre( 1, 1 ) );
}

TEST( SafetyPipeline, HoldIsOffByDefault )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  pipeline.prepare( { 0.0, 0.0 }, measured, false );
  EXPECT_THAT( pipeline.heldJoints(), ::testing::ElementsAre( 0, 0 ) );
}

// ---------------------------------------------------------------------------
// Velocity mode: the reference is a joint velocity demand and the hardware
// integrates the commanded velocity
// ---------------------------------------------------------------------------

namespace
{
Pipeline::Config makeVelocityConfig( const size_t n = 2 )
{
  auto cfg = makeConfig( n );
  cfg.velocity_mode = true;
  return cfg;
}

/// One cycle in velocity mode: the hardware integrates the commanded velocity exactly.
Pipeline::Events velocityCycle( Pipeline &pipeline, const std::vector<double> &reference,
                                std::vector<double> &measured,
                                const Pipeline::CollisionObservation &obs = {},
                                const bool bypass = false )
{
  pipeline.prepare( reference, measured, bypass );
  const auto events = pipeline.step( obs );
  for ( size_t i = 0; i < measured.size(); ++i ) {
    measured[i] += pipeline.velocity()[static_cast<Eigen::Index>( i )] * kDt;
  }
  return events;
}
} // namespace

TEST( SafetyPipeline, VelocityModeDropsThePositionOnlyBounds )
{
  auto cfg = makeVelocityConfig();
  cfg.reference_leash_time = 0.3;
  cfg.tracking_leash = 0.5;
  cfg.deviation_limits.assign( 2, 0.25 );
  Pipeline pipeline( cfg );

  EXPECT_EQ( pipeline.config().reference_leash_time, 0.0 );
  EXPECT_EQ( pipeline.config().tracking_leash, 0.0 );
  EXPECT_THAT( pipeline.config().deviation_limits, ::testing::Each( 0.0 ) );

  // The continuous-joint leash bound guards a lag measurement velocity mode never makes.
  auto continuous = makeVelocityConfig( 1 );
  continuous.joints[0].type = spc::JointType::CONTINUOUS;
  continuous.tracking_leash = 2.0;
  EXPECT_NO_THROW( Pipeline{ continuous } );
}

TEST( SafetyPipeline, VelocityModeTracksTheReferenceWithinBoxes )
{
  Pipeline pipeline( makeVelocityConfig() );
  std::vector<double> measured{ 0.0, 0.0 };

  double prev_v = 0.0, max_dv = 0.0;
  for ( int i = 0; i < 100; ++i ) {
    const double measured_before = measured[0];
    velocityCycle( pipeline, { 0.5, 0.0 }, measured );
    const double v = pipeline.velocity()[0];
    max_dv = std::max( max_dv, std::abs( v - prev_v ) );
    EXPECT_NEAR( pipeline.commandedPositions()[0], measured_before + prev_v * kDt, 1e-9 )
        << "the collision check must run one step ahead of the measurement";
    prev_v = v;
  }
  EXPECT_NEAR( pipeline.velocity()[0], 0.5, 1e-6 );
  EXPECT_NEAR( pipeline.velocity()[1], 0.0, 1e-9 );
  EXPECT_LE( max_dv, kAcc * kDt + 1e-9 ) << "the acceleration box still applies";

  // A demand above the joint's limit is clamped where it enters, not left for the QP's
  // own velocity box to cut down.
  for ( int i = 0; i < 100; ++i ) { velocityCycle( pipeline, { 5.0, 0.0 }, measured ); }
  EXPECT_NEAR( pipeline.qpInput().v_des[0], kVMax, 1e-9 );
  EXPECT_NEAR( pipeline.velocity()[0], kVMax, 1e-6 );
}

TEST( SafetyPipeline, VelocityModeReachesTheVelocityLimitDespiteDeviationLimits )
{
  // A configured deviation box must not reach the QP. Centred on the command it would
  // bound nothing, since the command is re-centred on the measurement every cycle, but
  // its braking envelope would cap the speed at sqrt(2 * a_dec * limit) ~ 0.58 rad/s.
  auto cfg = makeVelocityConfig( 1 );
  cfg.deviation_limits.assign( 1, 0.01 );
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 };
  for ( int i = 0; i < 200; ++i ) { velocityCycle( pipeline, { 1.5 }, measured ); }
  EXPECT_NEAR( pipeline.velocity()[0], kVMax, 1e-6 );
}

TEST( SafetyPipeline, VelocityModeFollowsAMeasurementThatJumps )
{
  // The hardware integrates, so the command has no lag to bound and cannot run away from
  // the joint: a jump (backdrive, a re-homed encoder) is simply where the robot now is,
  // and the rebase has to take it as such rather than brake or report a divergence.
  auto cfg = makeVelocityConfig( 1 );
  cfg.tracking_leash = 0.5;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0 };
  for ( int i = 0; i < 100; ++i ) { velocityCycle( pipeline, { 0.5 }, measured ); }
  ASSERT_NEAR( pipeline.velocity()[0], 0.5, 1e-6 );

  measured[0] += 1.5; // > 2 * tracking_leash: a position-mode divergence
  const double jumped_to = measured[0];
  velocityCycle( pipeline, { 0.5 }, measured );
  EXPECT_FALSE( pipeline.measurementDiverged() );
  EXPECT_NEAR( pipeline.velocity()[0], 0.5, 1e-6 ) << "the jump must not brake the demand";
  EXPECT_NEAR( pipeline.commandedPositions()[0], jumped_to + 0.5 * kDt, 1e-6 )
      << "the checked configuration follows the measurement";
}

TEST( SafetyPipeline, VelocityModeNaNReferenceBrakesAtTheDecelerationLimit )
{
  Pipeline pipeline( makeVelocityConfig( 1 ) );
  std::vector<double> measured{ 0.0 };
  for ( int i = 0; i < 200; ++i ) { velocityCycle( pipeline, { 1.0 }, measured ); }
  ASSERT_NEAR( pipeline.velocity()[0], kVMax, 1e-6 );

  double prev_v = pipeline.velocity()[0];
  for ( int i = 0; i < 50; ++i ) {
    velocityCycle( pipeline, { kNaN }, measured );
    const double v = pipeline.velocity()[0];
    EXPECT_LE( prev_v - v, kDec * kDt + 1e-9 ) << "no reference is a brake, not a stop";
    EXPECT_GE( v, -1e-9 );
    prev_v = v;
    ASSERT_TRUE( pipeline.lastResult().solved ) << "the QP must never be fed a NaN demand";
  }
  EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-9 );
  EXPECT_FALSE( pipeline.wantsMotion() );

  // An infinity is no demand either. Clamped to the velocity limit it would be full speed.
  for ( int i = 0; i < 20; ++i ) {
    velocityCycle( pipeline, { std::numeric_limits<double>::infinity() }, measured );
    ASSERT_TRUE( pipeline.lastResult().solved );
  }
  EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-9 );
}

TEST( SafetyPipeline, VelocityModeBrakesBeforeThePositionLimit )
{
  Pipeline pipeline( makeVelocityConfig( 1 ) ); // limits -3 .. 3
  std::vector<double> measured{ 0.0 };
  for ( int i = 0; i < 1000; ++i ) {
    velocityCycle( pipeline, { 1.0 }, measured );
    ASSERT_LE( measured[0], 3.0 + 1e-6 ) << "cycle " << i;
  }
  EXPECT_NEAR( measured[0], 3.0, 1e-3 );
  EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-3 );
}

TEST( SafetyPipeline, VelocityModeHeadOnBlockStallsAndParks )
{
  Pipeline pipeline( makeVelocityConfig() );
  std::vector<double> measured{ 0.0, 0.0 };

  Eigen::VectorXd gradient( 2 );
  gradient << -1.0, 0.0;
  std::vector<Pipeline::PairCandidate> pairs{ { 0.0, &gradient, 0 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  int stalled_at = -1, parked_at = -1;
  for ( int i = 0; i < 100; ++i ) {
    const auto events = velocityCycle( pipeline, { 1.0, 0.0 }, measured, obs );
    if ( events.stall.stalled && stalled_at < 0 ) {
      stalled_at = i;
    }
    if ( events.stall.parked && parked_at < 0 ) {
      parked_at = i;
    }
  }
  EXPECT_EQ( stalled_at, 19 ) << "stall_timeout = 0.2 s = 20 cycles";
  EXPECT_EQ( parked_at, 49 ) << "park_timeout = 0.5 s = 50 cycles";
  EXPECT_LT( std::abs( measured[0] ), 0.05 );

  // Parked: the demand is abandoned even when the blockage clears.
  for ( int i = 0; i < 20; ++i ) { velocityCycle( pipeline, { 1.0, 0.0 }, measured ); }
  EXPECT_TRUE( pipeline.parked() );
  EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-9 );
}

TEST( SafetyPipeline, VelocityModeParkReleasesOnlyAfterTheDemandPassedThroughZero )
{
  // A velocity reference is a demand, not a target: releasing on "the number changed"
  // would resume the moment a held stick wobbled, which is the opposite of parking.
  Pipeline pipeline( makeVelocityConfig( 1 ) );
  std::vector<double> measured{ 0.0 };

  pipeline.invalidate();
  velocityCycle( pipeline, { 0.5 }, measured );
  ASSERT_TRUE( pipeline.parked() );

  for ( int i = 0; i < 50; ++i ) {
    const double held = 0.5 + ( i % 2 ? 0.1 : -0.1 ); // a hand on the stick
    EXPECT_FALSE( pipeline.prepare( { held }, measured, false ) ) << "cycle " << i;
    pipeline.step( {} );
    ASSERT_NEAR( pipeline.velocity()[0], 0.0, 1e-9 ) << "cycle " << i;
  }
  EXPECT_TRUE( pipeline.parked() );

  velocityCycle( pipeline, { 0.0 }, measured ); // let go
  EXPECT_TRUE( pipeline.parked() ) << "letting go is not a new command by itself";
  EXPECT_TRUE( pipeline.prepare( { 0.5 }, measured, false ) ) << "asking again is";
  pipeline.step( {} );
  EXPECT_FALSE( pipeline.parked() );
  for ( int i = 0; i < 100; ++i ) { velocityCycle( pipeline, { 0.5 }, measured ); }
  EXPECT_NEAR( pipeline.velocity()[0], 0.5, 1e-6 );
}

TEST( SafetyPipeline, VelocityModeNonFiniteDemandDoesNotReleaseAPark )
{
  Pipeline pipeline( makeVelocityConfig( 1 ) );
  std::vector<double> measured{ 0.0 };
  pipeline.invalidate();
  velocityCycle( pipeline, { 0.5 }, measured );
  ASSERT_TRUE( pipeline.parked() );
  velocityCycle( pipeline, { 0.0 }, measured ); // let go

  for ( const double glitch : { std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity(), kNaN } ) {
    EXPECT_FALSE( pipeline.prepare( { glitch }, measured, false ) ) << glitch;
    pipeline.step( {} );
    EXPECT_TRUE( pipeline.parked() );
    EXPECT_NEAR( pipeline.velocity()[0], 0.0, 1e-9 );
  }
}

TEST( SafetyPipeline, VelocityModeParksAgainAfterResumingIntoTheSameBlock )
{
  // The second park is the one that chatters if letting go is not required again: the
  // stick is still held, so a release rule that only asks "is a demand being made" would
  // release on the very next cycle and push into the obstacle forever.
  Pipeline pipeline( makeVelocityConfig( 1 ) );
  std::vector<double> measured{ 0.0 };

  Eigen::VectorXd gradient( 1 );
  gradient << -1.0;
  std::vector<Pipeline::PairCandidate> pairs{ { 0.0, &gradient, 0 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  for ( int i = 0; i < 60; ++i ) { velocityCycle( pipeline, { 1.0 }, measured, obs ); }
  ASSERT_TRUE( pipeline.parked() );
  velocityCycle( pipeline, { 0.0 }, measured, obs );
  ASSERT_TRUE( pipeline.prepare( { 1.0 }, measured, obs.checks_active ) );
  pipeline.step( obs );
  ASSERT_FALSE( pipeline.parked() );

  // Straight back into the same block: it parks again and stays parked while held.
  for ( int i = 0; i < 60; ++i ) { velocityCycle( pipeline, { 1.0 }, measured, obs ); }
  ASSERT_TRUE( pipeline.parked() );
  for ( int i = 0; i < 200; ++i ) {
    EXPECT_FALSE( pipeline.prepare( { 1.0 }, measured, false ) ) << "cycle " << i;
    pipeline.step( obs );
    ASSERT_TRUE( pipeline.parked() ) << "cycle " << i;
  }
}

TEST( SafetyPipeline, VelocityModeAcceleratesFromACommandedStopNotFromTheLastSolve )
{
  // The caller stops the joints itself on a cycle it cannot run (busy handles, an
  // unreadable encoder). The acceleration box is anchored on the last velocity solved
  // for, so without being told, the first recovered cycle resumes at full speed against
  // a hardware that was just commanded to zero.
  Pipeline pipeline( makeVelocityConfig( 1 ) );
  std::vector<double> measured{ 0.0 };
  for ( int i = 0; i < 200; ++i ) { velocityCycle( pipeline, { 1.0 }, measured ); }
  ASSERT_NEAR( pipeline.velocity()[0], kVMax, 1e-6 );

  pipeline.noteCommandedStop(); // what the caller wrote instead of a cycle
  velocityCycle( pipeline, { 1.0 }, measured );
  EXPECT_LE( pipeline.velocity()[0], kAcc * kDt + 1e-9 )
      << "one acceleration step away from the stop that was written";
  EXPECT_FALSE( pipeline.parked() ) << "a stop the caller wrote abandons no reference";
}

TEST( SafetyPipeline, VelocityModeHoldPinsTheJointWithNoDemand )
{
  auto cfg = makeVelocityConfig();
  cfg.hold_unrequested = true;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 0.0, 0.0 };
  const Eigen::VectorXd gradient = sweepGradient(); // escapes by sweeping joint 1 aside
  std::vector<Pipeline::PairCandidate> pairs{ { 0.001, &gradient, 0 } };
  Pipeline::CollisionObservation obs;
  obs.checks_active = true;
  obs.state_valid = true;
  obs.pairs = &pairs;

  for ( int i = 0; i < 50; ++i ) { velocityCycle( pipeline, { 1.0, 0.0 }, measured, obs ); }
  EXPECT_THAT( pipeline.heldJoints(), ::testing::ElementsAre( 0, 1 ) );
  EXPECT_EQ( measured[1], 0.0 ) << "a joint with no demand must not be swept aside";
}

TEST( SafetyPipeline, VelocityModeIsIndifferentToAWrappedMeasurement )
{
  // Nothing in velocity mode measures a distance between two angles, so a hardware that
  // reports wrapped positions cannot produce a phantom 2*pi to travel or brake for.
  auto cfg = makeVelocityConfig( 1 );
  cfg.joints[0].type = spc::JointType::CONTINUOUS;
  cfg.joints[0].has_position_limits = false;
  Pipeline pipeline( cfg );

  std::vector<double> measured{ 3.13 };
  for ( int i = 0; i < 100; ++i ) {
    velocityCycle( pipeline, { 1.0 }, measured );
    if ( measured[0] > M_PI ) {
      measured[0] -= 2.0 * M_PI; // the hardware wraps
    }
    ASSERT_NEAR( pipeline.velocity()[0], std::min( kVMax, ( i + 1 ) * kAcc * kDt ), 1e-6 )
        << "cycle " << i;
  }
  EXPECT_FALSE( pipeline.measurementDiverged() );
}

int main( int argc, char **argv )
{
  testing::InitGoogleMock( &argc, argv );
  return RUN_ALL_TESTS();
}
