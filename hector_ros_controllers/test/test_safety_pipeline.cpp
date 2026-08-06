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

TEST( SafetyPipeline, InvalidateRebasesToMeasuredState )
{
  Pipeline pipeline( makeConfig() );
  std::vector<double> measured{ 0.0, 0.0 };
  for ( int i = 0; i < 50; ++i ) { cycle( pipeline, { 0.5, 0.0 }, measured ); }
  EXPECT_GT( measured[0], 0.1 );

  // E-stop style: state invalidated, robot ends up somewhere else
  pipeline.invalidate();
  measured = { 1.0, -0.5 };
  cycle( pipeline, { kNaN, kNaN }, measured );
  EXPECT_NEAR( measured[0], 1.0, 1e-9 );
  EXPECT_NEAR( measured[1], -0.5, 1e-9 );
  EXPECT_NEAR( pipeline.velocity().cwiseAbs().maxCoeff(), 0.0, 1e-9 );
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

  // Bypass drops the deviation boxes → only the position limits remain
  pipeline.prepare( reference, measured, true );
  EXPECT_NEAR( pipeline.qpInput().q_hi[0], 3.0, 1e-9 );
  EXPECT_NEAR( pipeline.qpInput().q_lo[0], -3.0, 1e-9 );
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

int main( int argc, char **argv )
{
  testing::InitGoogleMock( &argc, argv );
  return RUN_ALL_TESTS();
}
