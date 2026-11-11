//
// Unit tests for SafetyQpLimiter — the velocity-damper QP at the core of the
// SafetyPositionController. Pure Eigen/proxsuite, no ROS.
//
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <proxsuite/proxqp/dense/dense.hpp>

#include "safety_position_controller/safety_qp_limiter.hpp"

using safety_position_controller::QpCollisionConstraint;
using safety_position_controller::SafetyQpInput;
using safety_position_controller::SafetyQpLimiter;
using safety_position_controller::SafetyQpParams;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

SafetyQpParams makeParams( const int n, const double v_max = 1.0, const double a_acc = 100.0,
                           const double a_dec = 100.0 )
{
  SafetyQpParams p;
  p.dt = 0.01;
  p.v_max = Eigen::VectorXd::Constant( n, v_max );
  p.a_acc = Eigen::VectorXd::Constant( n, a_acc );
  p.a_dec = Eigen::VectorXd::Constant( n, a_dec );
  p.damper_xi = 2.0;
  p.d_pad = 0.01;
  p.d_zone = 0.05;
  p.max_repulsion_speed = 0.05;
  p.max_collision_constraints = 5;
  return p;
}

SafetyQpInput makeInput( const int n )
{
  SafetyQpInput in;
  in.v_des = Eigen::VectorXd::Zero( n );
  in.v_prev = Eigen::VectorXd::Zero( n );
  in.q = Eigen::VectorXd::Zero( n );
  in.q_lo = Eigen::VectorXd::Constant( n, -kInf );
  in.q_hi = Eigen::VectorXd::Constant( n, kInf );
  return in;
}

QpCollisionConstraint makeCollision( const Eigen::VectorXd &normal, const double distance )
{
  QpCollisionConstraint c;
  c.normal = normal;
  c.distance = distance;
  return c;
}
} // namespace

// ---------------------------------------------------------------------------
// Construction / validation
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, ConstructorValidatesParams )
{
  EXPECT_THROW( SafetyQpLimiter( 0, makeParams( 1 ) ), std::invalid_argument );
  EXPECT_THROW( SafetyQpLimiter( 3, makeParams( 2 ) ), std::invalid_argument );

  auto bad_dt = makeParams( 2 );
  bad_dt.dt = 0.0;
  EXPECT_THROW( SafetyQpLimiter( 2, bad_dt ), std::invalid_argument );

  auto bad_zone = makeParams( 2 );
  bad_zone.d_zone = bad_zone.d_pad;
  EXPECT_THROW( SafetyQpLimiter( 2, bad_zone ), std::invalid_argument );

  EXPECT_NO_THROW( SafetyQpLimiter( 2, makeParams( 2 ) ) );
}

// ---------------------------------------------------------------------------
// Free space
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, FreeSpaceTracksDesiredExactly )
{
  SafetyQpLimiter limiter( 3, makeParams( 3 ) );
  auto in = makeInput( 3 );
  in.v_des << 0.5, -0.3, 0.9;
  in.v_prev = in.v_des; // acceleration not binding

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_FALSE( res.braking );
  EXPECT_LT( ( res.v - in.v_des ).norm(), 1e-5 );
}

TEST( SafetyQpLimiter, VelocityLimitClamps )
{
  SafetyQpLimiter limiter( 2, makeParams( 2, /*v_max=*/1.0 ) );
  auto in = makeInput( 2 );
  in.v_des << 5.0, -5.0;
  in.v_prev << 1.0, -1.0; // already at the velocity limit

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_NEAR( res.v[0], 1.0, 1e-6 );
  EXPECT_NEAR( res.v[1], -1.0, 1e-6 );
}

// ---------------------------------------------------------------------------
// Acceleration limits (the original jump bug, at unit level)
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, AccelerationLimitRampsFromStandstill )
{
  const double a_acc = 5.0, dt = 0.01, v_max = 1.0;
  auto params = makeParams( 1, v_max, a_acc, /*a_dec=*/50.0 );
  SafetyQpLimiter limiter( 1, params );

  auto in = makeInput( 1 );
  in.v_des << v_max;
  in.v_prev << 0.0;

  double v = 0.0;
  for ( int cycle = 0; cycle < 200 && v < v_max - 1e-9; ++cycle ) {
    in.v_prev << v;
    const auto res = limiter.solve( in );
    ASSERT_TRUE( res.solved );
    // Per-cycle velocity change bounded by the acceleration limit
    EXPECT_LE( res.v[0] - v, a_acc * dt + 1e-6 ) << "cycle " << cycle;
    EXPECT_GE( res.v[0], v - 1e-9 ) << "must not decelerate on the way up";
    v = res.v[0];
  }
  EXPECT_NEAR( v, v_max, 1e-6 ); // target speed is reached, just later
}

TEST( SafetyQpLimiter, DecelerationUsesSeparateLimit )
{
  const double a_acc = 5.0, a_dec = 20.0, dt = 0.01;
  SafetyQpLimiter limiter( 1, makeParams( 1, 1.0, a_acc, a_dec ) );

  auto in = makeInput( 1 );
  in.v_des << 0.0;
  in.v_prev << 1.0;

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  // Braking down from +1.0: allowed step is a_dec * dt, not a_acc * dt
  EXPECT_NEAR( res.v[0], 1.0 - a_dec * dt, 1e-6 );
}

// ---------------------------------------------------------------------------
// Position limits
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, PositionLimitNeverOvershot )
{
  const double a_acc = 10.0, a_dec = 10.0, v_max = 1.0, dt = 0.01;
  auto params = makeParams( 1, v_max, a_acc, a_dec );
  SafetyQpLimiter limiter( 1, params );

  auto in = makeInput( 1 );
  in.q_hi << 0.5;
  in.v_des << v_max;

  // Simulate: command integrates the QP velocity; desired velocity keeps pushing up.
  double q = 0.0, v = 0.0;
  for ( int cycle = 0; cycle < 2000; ++cycle ) {
    in.q << q;
    in.v_prev << v;
    in.v_des << SafetyQpLimiter::desiredVelocity( 0.5 - q, v_max, a_dec, dt );
    const auto res = limiter.solve( in );
    v = res.v[0];
    q += v * dt;
    ASSERT_LE( q, 0.5 + 1e-9 ) << "position limit overshot at cycle " << cycle;
    if ( std::abs( 0.5 - q ) < 1e-6 && std::abs( v ) < 1e-6 ) {
      break;
    }
  }
  EXPECT_NEAR( q, 0.5, 1e-4 ); // converged to the limit without crossing it
}

// ---------------------------------------------------------------------------
// Collision dampers: flow-around behavior
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, BlockedDirectionSlidesTangentially )
{
  // Constraint at the padding: normal (1,0) → v_x >= 0 (x-motion toward obstacle is blocked).
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << -0.8, -0.5; // wants to move INTO the obstacle and sideways
  in.v_prev = in.v_des;   // acceleration not binding
  in.collisions.push_back(
      makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), /*distance=*/0.01 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_EQ( res.num_collision_constraints, 1 );
  EXPECT_NEAR( res.v[0], 0.0, 1e-5 );  // blocked component projected out
  EXPECT_NEAR( res.v[1], -0.5, 1e-5 ); // tangential component flows through
}

TEST( SafetyQpLimiter, DamperLimitsApproachSpeedInsideZone )
{
  auto params = makeParams( 1 );
  SafetyQpLimiter limiter( 1, params );
  auto in = makeInput( 1 );
  in.v_des << -1.0; // approach at full speed
  in.v_prev << -1.0;

  const double d = 0.03; // halfway between pad (0.01) and zone (0.05)
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 1 ) << 1.0 ).finished(), d ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  // allowed approach speed: xi * (d - pad)/(zone - pad) = 2.0 * 0.5 = 1.0 m/s per unit gradient
  const double allowed = params.damper_xi * ( d - params.d_pad ) / ( params.d_zone - params.d_pad );
  EXPECT_NEAR( res.v[0], -allowed, 1e-5 );
}

TEST( SafetyQpLimiter, PenetrationCausesCappedPushOut )
{
  auto params = makeParams( 1 );
  SafetyQpLimiter limiter( 1, params );
  auto in = makeInput( 1 );
  in.v_des << 0.0; // no desire to move — push-out must come from the damper
  in.collisions.push_back(
      makeCollision( ( Eigen::VectorXd( 1 ) << 1.0 ).finished(), /*distance=*/0.0 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  // rhs = min(xi * (pad - d)/(zone - pad), cap) = min(2.0*0.25, 0.05) = 0.05
  EXPECT_NEAR( res.v[0], params.max_repulsion_speed, 1e-5 );
}

TEST( SafetyQpLimiter, OpposingConstraintsStopOnlyThatAxis )
{
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.7, 0.4;
  in.v_prev = in.v_des;
  // Two opposing walls at the padding on the x axis: v_x >= 0 and v_x <= 0
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), 0.01 ) );
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << -1.0, 0.0 ).finished(), 0.01 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_NEAR( res.v[0], 0.0, 1e-5 ); // pinned axis stops
  EXPECT_NEAR( res.v[1], 0.4, 1e-5 ); // free axis keeps moving
}

TEST( SafetyQpLimiter, ExcessConstraintsAreDropped )
{
  auto params = makeParams( 2 );
  params.max_collision_constraints = 2;
  SafetyQpLimiter limiter( 2, params );
  auto in = makeInput( 2 );
  in.v_des << 0.1, 0.1;
  for ( int k = 0; k < 5; ++k ) {
    in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), 0.04 ) );
  }
  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_EQ( res.num_collision_constraints, 2 );
}

// ---------------------------------------------------------------------------
// Wedged: opposing push-out demands → no-worsen relaxation, escape stays possible
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, WedgedRelaxesPushOutAndAllowsEscape )
{
  // Opposing penetrating contacts demand incompatible push-outs → full QP infeasible.
  // Stage 2 relaxes to "no-worsen": wedged axis pinned, free axis still escapes.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.6, 0.4;  // x is wedged, y is free
  in.v_prev << 0.0, 0.0; // standstill: the old pure-braking fallback would stay frozen
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), -0.02 ) );
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << -1.0, 0.0 ).finished(), -0.02 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_TRUE( res.push_out_relaxed );
  EXPECT_FALSE( res.braking );
  EXPECT_NEAR( res.v[0], 0.0, 1e-5 ); // wedged axis: not deeper in either direction
  EXPECT_NEAR( res.v[1], 0.4, 1e-5 ); // free axis: escape motion flows through
}

TEST( SafetyQpLimiter, WedgedOutwardMotionIsAllowed )
{
  // Same wedge on x, plus a third penetrating contact blocking -y. Desired motion +y
  // points OUT of that contact → allowed at full speed under the relaxation.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.0, 0.8;
  in.v_prev << 0.0, 0.8; // acceleration not binding
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), -0.02 ) );
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << -1.0, 0.0 ).finished(), -0.02 ) );
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 0.0, 1.0 ).finished(), -0.01 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_TRUE( res.push_out_relaxed );
  EXPECT_NEAR( res.v[1], 0.8, 1e-5 ); // moving out of the -y contact is unrestricted
}

// ---------------------------------------------------------------------------
// Contact crawl: bounded tangential speed inside the padding (linearization sag)
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, ContactCrawlClampsTangentialSpeedInsidePadding )
{
  auto params = makeParams( 2 );
  params.contact_crawl_speed = 0.2;
  SafetyQpLimiter limiter( 2, params );

  auto in = makeInput( 2 );
  in.v_des << 0.0, 0.9; // fast tangential grind along the contact
  in.v_prev << 0.0, 0.2;
  // Pair inside the padding (0.005 < 0.01) → crawl engages; push-out demanded on x
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), 0.005 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_NEAR( res.v[0], params.max_repulsion_speed, 1e-5 ); // push-out still happens
  EXPECT_NEAR( res.v[1], 0.2, 1e-5 ); // tangential speed clamped to the crawl, not 0.9
}

TEST( SafetyQpLimiter, ContactCrawlDeceleratesFromEntrySpeed )
{
  const double a_dec = 20.0, dt = 0.01;
  auto params = makeParams( 2, 2.0, /*a_acc=*/10.0, a_dec );
  params.contact_crawl_speed = 0.2;
  SafetyQpLimiter limiter( 2, params );

  auto in = makeInput( 2 );
  in.v_des << 0.0, 1.0;
  in.v_prev << 0.0, 1.0; // entered the padding still fast
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), 0.002 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_FALSE( res.bounds_conflict ); // crawl clamp decays, it must not conflict
  // Decaying clamp: max(crawl, v_prev - a_dec*dt) = max(0.2, 0.8) = 0.8
  EXPECT_NEAR( res.v[1], 1.0 - a_dec * dt, 1e-5 );

  // Repeated cycles converge to the crawl speed
  double v = res.v[1];
  for ( int i = 0; i < 20; ++i ) {
    in.v_prev << 0.0, v;
    v = limiter.solve( in ).v[1];
  }
  EXPECT_NEAR( v, params.contact_crawl_speed, 1e-5 );
}

TEST( SafetyQpLimiter, ContactCrawlInactiveOutsidePadding )
{
  auto params = makeParams( 1 );
  params.contact_crawl_speed = 0.2;
  SafetyQpLimiter limiter( 1, params );

  auto in = makeInput( 1 );
  in.v_des << 0.8;
  in.v_prev << 0.8;
  // In the zone but outside the padding → no crawl (tangential pair: zero gradient on
  // the moving joint would be skipped upstream; here use an orthogonal-ish weak normal)
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 1 ) << 0.01 ).finished(), 0.03 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_NEAR( res.v[0], 0.8, 1e-4 );
}

// ---------------------------------------------------------------------------
// Deepest-pair push-out priority (stage 2a)
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, DeepestPairKeepsPushOutInPocket )
{
  // y axis is wedged by two opposing penetrating contacts (infeasible together), while
  // the DEEPEST contact is on x. Stage 2a must keep the deepest push-out: the arm
  // extracts from the worst contact instead of parking inside it.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.0, 0.0;
  in.collisions.push_back(
      makeCollision( ( Eigen::VectorXd( 2 ) << 1.0, 0.0 ).finished(), -0.02 ) ); // deepest
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 0.0, 1.0 ).finished(), -0.005 ) );
  in.collisions.push_back( makeCollision( ( Eigen::VectorXd( 2 ) << 0.0, -1.0 ).finished(), -0.004 ) );

  const auto res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_TRUE( res.push_out_relaxed );
  EXPECT_NEAR( res.v[0], 0.05, 1e-5 ); // deepest push-out preserved (capped speed)
  EXPECT_NEAR( res.v[1], 0.0, 1e-5 );  // wedged axis pinned
}

// ---------------------------------------------------------------------------
// Infeasibility → braking fallback
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, InfeasibleFallsBackToBraking )
{
  // Deep penetration demands push-out, but the acceleration limit makes any
  // sufficient velocity unreachable this cycle → infeasible → brake.
  const double a_acc = 0.1, a_dec = 0.5, dt = 0.01;
  SafetyQpLimiter limiter( 1, makeParams( 1, 1.0, a_acc, a_dec ) );
  auto in = makeInput( 1 );
  in.v_des << 0.0;
  in.v_prev << -0.4; // still moving toward the obstacle
  in.collisions.push_back(
      makeCollision( ( Eigen::VectorXd( 1 ) << 1.0 ).finished(), /*distance=*/-0.01 ) );

  const auto res = limiter.solve( in );
  EXPECT_TRUE( res.braking );
  EXPECT_FALSE( res.solved );
  // Braking: velocity magnitude shrinks by a_dec*dt toward zero
  EXPECT_NEAR( res.v[0], -0.4 + a_dec * dt, 1e-9 );

  // Repeated braking converges to zero velocity
  auto v = res.v[0];
  for ( int i = 0; i < 200 && std::abs( v ) > 1e-12; ++i ) {
    in.v_prev << v;
    v = limiter.solve( in ).v[0];
  }
  EXPECT_NEAR( v, 0.0, 1e-9 );
}

// ---------------------------------------------------------------------------
// Optimality against an independent solver
// ---------------------------------------------------------------------------

namespace
{
/// Damper row bound before any relaxation, as computed by the limiter.
double unrelaxedCollisionRhs( const SafetyQpParams &params, const QpCollisionConstraint &c )
{
  const double damper =
      -params.damper_xi * ( c.distance - params.d_pad ) / ( params.d_zone - params.d_pad );
  const double dist = c.distance - params.d_pad;
  double braking = 0.0;
  if ( dist > 0.0 ) {
    const double a = std::max( c.normal.cwiseAbs().dot( params.a_dec ), 1e-9 );
    const double half_step = 0.5 * a * params.dt;
    braking = -std::min( dist / params.dt,
                         -half_step + std::sqrt( half_step * half_step + 2.0 * a * dist ) );
  }
  return std::min( std::max( damper, braking ), params.max_repulsion_speed );
}

/// Lawson-Hanson: residual of min ||A x - b|| subject to x >= 0.
double nonNegativeLeastSquaresResidual( const Eigen::MatrixXd &A, const Eigen::VectorXd &b )
{
  const auto m = A.cols();
  Eigen::VectorXd x = Eigen::VectorXd::Zero( m );
  std::vector<bool> passive( static_cast<std::size_t>( m ), false );
  for ( int outer = 0; outer < 3 * m + 10; ++outer ) {
    const Eigen::VectorXd w = A.transpose() * ( b - A * x );
    Eigen::Index enter = -1;
    for ( Eigen::Index k = 0; k < m; ++k ) {
      if ( !passive[k] && w[k] > 1e-14 && ( enter < 0 || w[k] > w[enter] ) ) {
        enter = k;
      }
    }
    if ( enter < 0 ) {
      break;
    }
    passive[enter] = true;
    while ( true ) {
      std::vector<Eigen::Index> P;
      for ( Eigen::Index k = 0; k < m; ++k ) {
        if ( passive[k] ) {
          P.push_back( k );
        }
      }
      const Eigen::VectorXd z = A( Eigen::all, P ).completeOrthogonalDecomposition().solve( b );
      double alpha = 1.0;
      for ( std::size_t i = 0; i < P.size(); ++i ) {
        if ( z[i] <= 0.0 ) {
          alpha = std::min( alpha, x[P[i]] / ( x[P[i]] - z[i] ) );
        }
      }
      for ( std::size_t i = 0; i < P.size(); ++i ) {
        x[P[i]] += alpha * ( z[i] - x[P[i]] );
        if ( alpha < 1.0 && x[P[i]] <= 1e-15 ) {
          x[P[i]] = 0.0;
          passive[P[i]] = false;
        }
      }
      if ( alpha == 1.0 ) {
        break;
      }
    }
  }
  return ( A * x - b ).norm();
}

/// KKT residual of v for min 0.5 * s * ||v||^2 - v_des^T v s.t. l <= C v <= u: how far
/// s * v - v_des is from the cone of the normals of the rows that are tight at v.
double kktResidual( const Eigen::VectorXd &v, const Eigen::VectorXd &v_des, const double s,
                    const Eigen::MatrixXd &C, const Eigen::VectorXd &l, const Eigen::VectorXd &u )
{
  constexpr double kTight = 1e-9;
  const Eigen::VectorXd cv = C * v;
  Eigen::MatrixXd normals( v.size(), 2 * C.rows() );
  Eigen::Index count = 0;
  for ( Eigen::Index i = 0; i < C.rows(); ++i ) {
    if ( cv[i] - l[i] <= kTight ) {
      normals.col( count++ ) = C.row( i ).transpose();
    }
    if ( u[i] - cv[i] <= kTight ) {
      normals.col( count++ ) = -C.row( i ).transpose();
    }
  }
  return nonNegativeLeastSquaresResidual( normals.leftCols( count ), s * v - v_des );
}

/// Smallest achievable max violation of the damper rows (rows n..) of l <= C v <= u
/// within the box rows (rows 0..n), from a proxqp solve that is feasible by construction:
/// min 0.5 * ||t||^2 s.t. box, C_k v + t_k >= l_k, t >= 0. Positive iff the QP is
/// infeasible; -1 if proxqp fails, which no infeasibility claim may rely on.
double minimumDamperViolation( const Eigen::Index n, const Eigen::MatrixXd &C,
                               const Eigen::VectorXd &l, const Eigen::VectorXd &u )
{
  const auto m = C.rows() - n;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero( n + m, n + m );
  H.bottomRightCorner( m, m ).setIdentity();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero( n + 2 * m, n + m );
  Eigen::VectorXd lower( n + 2 * m ), upper( n + 2 * m );
  A.topLeftCorner( n + m, n ) = C;
  A.block( n, n, m, m ).setIdentity();
  A.bottomRightCorner( m, m ).setIdentity();
  lower << l, Eigen::VectorXd::Zero( m );
  upper << u, Eigen::VectorXd::Constant( m, kInf );
  proxsuite::proxqp::dense::QP<double> qp( n + m, 0, n + 2 * m );
  qp.settings.eps_abs = 1e-10;
  qp.init( H, Eigen::VectorXd::Zero( n + m ), std::nullopt, std::nullopt, A, lower, upper );
  qp.solve();
  if ( qp.results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ) {
    return -1.0;
  }
  return m == 0 ? 0.0 : qp.results.x.tail( m ).maxCoeff();
}
} // namespace

TEST( SafetyQpLimiter, SolutionIsOptimalAndRelaxesOnlyWhenInfeasible )
{
  // The command must be the optimum of the safe-set projection in every cycle: a
  // too-conservative solution slows the limb down near obstacles, and a solver that
  // wrongly reports infeasibility brakes it or gives up push-out. Each cycle's final QP
  // (box and relaxed damper rows as reported) must hold at the command together with the
  // KKT conditions, which make it the unique optimum. Infeasibility claims, including
  // every relaxation stage the limiter skipped past, are cross-checked with proxqp by
  // minimizing the damper violation.
  constexpr int n = 7;
  auto params = makeParams( n, 1.0, 5.0, 15.0 );
  params.dt = 0.02;
  params.d_pad = 0.03;
  params.d_zone = 0.12;
  params.max_collision_constraints = 64;
  SafetyQpLimiter limiter( n, params );
  std::mt19937 rng( 3 );
  std::uniform_real_distribution<double> unit( -1.0, 1.0 );
  const double hessian_scale = 1.0 + params.regularization;

  int solved = 0, relaxed = 0;
  for ( int cycle = 0; cycle < 2000; ++cycle ) {
    auto in = makeInput( n );
    in.v_des = Eigen::VectorXd::NullaryExpr( n, [&]() { return 1.5 * unit( rng ); } );
    in.v_prev = Eigen::VectorXd::NullaryExpr( n, [&]() { return 0.3 * unit( rng ); } );
    const int num_collisions = static_cast<int>( 40 * std::abs( unit( rng ) ) );
    for ( int k = 0; k < num_collisions; ++k ) {
      const Eigen::VectorXd normal =
          Eigen::VectorXd::NullaryExpr( n, [&]() { return 0.3 * unit( rng ); } );
      in.collisions.push_back( makeCollision( normal, 0.12 * std::abs( unit( rng ) ) ) );
    }
    const auto &res = limiter.solve( in );
    const int m = res.num_collision_constraints;

    Eigen::MatrixXd C( n + m, n );
    Eigen::VectorXd l( n + m ), u( n + m );
    C.topRows( n ).setIdentity();
    l.head( n ) = limiter.lastBoxLower();
    u.head( n ) = limiter.lastBoxUpper();
    for ( int k = 0; k < m; ++k ) {
      C.row( n + k ) = in.collisions[k].normal.transpose();
      l[n + k] = res.collision_rhs[k];
      u[n + k] = kInf;
    }

    if ( !res.solved ) {
      EXPECT_GT( minimumDamperViolation( n, C, l, u ), 1e-9 )
          << "cycle " << cycle << ": feasible QP reported infeasible";
      continue;
    }
    ++solved;
    const Eigen::VectorXd cv = C * res.v;
    EXPECT_LT( std::max( ( l - cv ).maxCoeff(), ( cv - u ).maxCoeff() ), 1e-9 ) << "cycle " << cycle;
    EXPECT_LT( kktResidual( res.v, in.v_des, hessian_scale, C, l, u ), 1e-9 ) << "cycle " << cycle;

    if ( res.push_out_relaxed ) {
      ++relaxed;
      // Stage 1: the unrelaxed problem.
      Eigen::VectorXd l_stage = l;
      int deepest = -1;
      for ( int k = 0; k < m; ++k ) {
        l_stage[n + k] = unrelaxedCollisionRhs( params, in.collisions[k] );
        if ( l_stage[n + k] > 0.0 &&
             ( deepest < 0 || in.collisions[k].distance < in.collisions[deepest].distance ) ) {
          deepest = k;
        }
      }
      EXPECT_GT( minimumDamperViolation( n, C, l_stage, u ), 1e-9 )
          << "cycle " << cycle << ": relaxed a feasible problem";
      // Stage 2a: only the deepest pair keeps its push-out. Skipped past iff the final
      // problem relaxed that pair too.
      ASSERT_GE( deepest, 0 );
      if ( res.collision_rhs[deepest] == 0.0 ) {
        const double deepest_rhs = l_stage[n + deepest];
        l_stage.tail( m ) = l_stage.tail( m ).cwiseMin( 0.0 );
        l_stage[n + deepest] = deepest_rhs;
        EXPECT_GT( minimumDamperViolation( n, C, l_stage, u ), 1e-9 )
            << "cycle " << cycle << ": dropped the deepest push-out of a feasible problem";
      }
    }
  }
  EXPECT_GT( solved, 1000 );
  EXPECT_GT( relaxed, 10 );
}

// ---------------------------------------------------------------------------
// Degenerate constraint sets with a known optimum
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, DuplicateAndNearParallelRowsKeepTheTightest )
{
  // Contacts of neighbouring geometry pairs often share a normal. Redundant rows must
  // neither be reported infeasible nor loosen the tightest of them.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << -0.8, 0.3;
  in.v_prev = in.v_des;
  for ( const double distance : { 0.03, 0.02, 0.03, 0.025 } ) {
    in.collisions.push_back( makeCollision( Eigen::Vector2d( 1.0, 0.0 ), distance ) );
  }
  in.collisions.push_back( makeCollision( Eigen::Vector2d( 1.0, 1e-9 ), 0.02 ) );
  in.collisions.push_back( makeCollision( Eigen::Vector2d( 1.0, -1e-9 ), 0.021 ) );

  const auto &res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_FALSE( res.push_out_relaxed );
  const double tightest = res.collision_rhs.maxCoeff();
  ASSERT_GT( tightest, in.v_des[0] );
  EXPECT_NEAR( res.v[0], tightest, 1e-8 );
  EXPECT_NEAR( res.v[1], 0.3, 1e-8 );
  EXPECT_GE( ( res.collision_velocity - res.collision_rhs ).minCoeff(), -1e-9 );
}

TEST( SafetyQpLimiter, ZeroNormalPushOutIsRelaxed )
{
  // A penetrating pair whose distance does not depend on the controlled joints demands
  // a push-out no velocity can deliver. That must relax to "no-worsen" (trivially met),
  // not brake the whole arm.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.4, -0.2;
  in.v_prev = in.v_des;
  in.collisions.push_back( makeCollision( Eigen::Vector2d::Zero(), -0.02 ) );

  const auto &res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_TRUE( res.push_out_relaxed );
  EXPECT_FALSE( res.braking );
  EXPECT_EQ( res.collision_rhs[0], 0.0 );
  EXPECT_NEAR( ( res.v - in.v_des ).norm(), 0.0, 1e-8 );
}

TEST( SafetyQpLimiter, KeepsExactlyMaxCollisionConstraintsRows )
{
  // With max_collision_constraints = 64, the 64th contact still constrains the motion
  // and the 65th is dropped.
  auto params = makeParams( 2 );
  params.max_collision_constraints = 64;
  SafetyQpLimiter limiter( 2, params );
  auto in = makeInput( 2 );
  in.v_des << -0.5, -0.5;
  in.v_prev = in.v_des;
  for ( int k = 0; k < 63; ++k ) {
    const double angle = 0.1 * k;
    in.collisions.push_back(
        makeCollision( 0.3 * Eigen::Vector2d( std::cos( angle ), std::sin( angle ) ), 0.2 ) );
  }
  in.collisions.push_back( makeCollision( Eigen::Vector2d( 1.0, 0.0 ), 0.015 ) );
  in.collisions.push_back( makeCollision( Eigen::Vector2d( 0.0, 1.0 ), 0.015 ) );

  const auto &res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  ASSERT_EQ( res.num_collision_constraints, 64 );
  ASSERT_GT( res.collision_rhs[63], in.v_des[0] );
  EXPECT_NEAR( res.v[0], res.collision_rhs[63], 1e-8 ) << "the 64th row binds";
  EXPECT_NEAR( res.v[1], -0.5, 1e-8 ) << "the 65th row is dropped";
}

TEST( SafetyQpLimiter, CollisionFlowsAroundPinnedJoints )
{
  // Held joints give equality boxes (lb == ub), at zero and mid-deceleration. A contact
  // coupling them to a free joint must be resolved by the free joint alone.
  auto params = makeParams( 3 );
  params.a_dec[1] = 20.0;
  SafetyQpLimiter limiter( 3, params );
  auto in = makeInput( 3 );
  in.v_des << -1.0, 0.0, 0.0;
  in.v_prev << -0.8, 0.5, 0.0;
  in.hold = { 0, 1, 1 };
  in.collisions.push_back( makeCollision( Eigen::Vector3d( 1.0, 1.0, 1.0 ), 0.02 ) );

  const auto &res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  const double pinned = 0.5 - params.a_dec[1] * params.dt;
  EXPECT_EQ( limiter.lastBoxLower()[1], limiter.lastBoxUpper()[1] );
  EXPECT_NEAR( limiter.lastBoxLower()[1], pinned, 1e-12 );
  EXPECT_EQ( res.v[1], pinned );
  EXPECT_EQ( res.v[2], 0.0 );
  ASSERT_GT( res.collision_rhs[0] - pinned, in.v_des[0] );
  EXPECT_NEAR( res.v[0], res.collision_rhs[0] - pinned, 1e-8 );
}

TEST( SafetyQpLimiter, DegenerateVertexWithMoreTightRowsThanJoints )
{
  // Four capped push-outs all meet at v = (0.05, 0.05): a vertex with more tight rows
  // than dimensions, where an active-set method can cycle or misjudge dependence.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << -0.5, -0.5;
  for ( const Eigen::Vector2d &normal :
        { Eigen::Vector2d( 1.0, 0.0 ), Eigen::Vector2d( 0.0, 1.0 ), Eigen::Vector2d( 0.5, 0.5 ),
          Eigen::Vector2d( 0.7, 0.3 ) } ) {
    in.collisions.push_back( makeCollision( normal, -0.005 ) );
  }

  const auto &res = limiter.solve( in );
  ASSERT_TRUE( res.solved );
  EXPECT_FALSE( res.push_out_relaxed );
  ASSERT_TRUE( ( res.collision_rhs.array() == 0.05 ).all() );
  EXPECT_NEAR( res.v[0], 0.05, 1e-9 );
  EXPECT_NEAR( res.v[1], 0.05, 1e-9 );
}

TEST( SafetyQpLimiter, FeasibleStandstillIsNeverReportedInfeasible )
{
  // Outside the padding every damper allows v = 0, and from a slow previous velocity so
  // does the acceleration box: the QP is feasible and must be solved without relaxing
  // or braking, also with duplicate, opposing and near-parallel contacts.
  constexpr int n = 7;
  auto params = makeParams( n, 1.0, 5.0, 15.0 );
  params.max_collision_constraints = 64;
  SafetyQpLimiter limiter( n, params );
  std::mt19937 rng( 11 );
  std::uniform_real_distribution<double> unit( -1.0, 1.0 );
  for ( int cycle = 0; cycle < 500; ++cycle ) {
    auto in = makeInput( n );
    in.v_des = Eigen::VectorXd::NullaryExpr( n, [&]() { return 1.5 * unit( rng ); } );
    in.v_prev = Eigen::VectorXd::NullaryExpr( n, [&]() { return 0.04 * unit( rng ); } );
    for ( int k = 0; k < 64; ++k ) {
      Eigen::VectorXd normal = Eigen::VectorXd::NullaryExpr( n, [&]() { return 0.3 * unit( rng ); } );
      if ( k > 0 && cycle % 4 == 1 ) {
        normal = in.collisions.back().normal;
      } else if ( k > 0 && cycle % 4 == 2 ) {
        normal = -in.collisions.back().normal;
      } else if ( k > 0 && cycle % 4 == 3 ) {
        normal = in.collisions.back().normal +
                 1e-9 * Eigen::VectorXd::NullaryExpr( n, [&]() { return unit( rng ); } );
      }
      const double distance = params.d_pad + ( k % 3 == 0 ? 0.0 : 0.05 * std::abs( unit( rng ) ) );
      in.collisions.push_back( makeCollision( normal, distance ) );
    }
    const auto &res = limiter.solve( in );
    ASSERT_TRUE( res.solved ) << "cycle " << cycle;
    EXPECT_FALSE( res.push_out_relaxed ) << "cycle " << cycle;
    EXPECT_GE( ( res.collision_velocity - res.collision_rhs ).minCoeff(), -1e-9 ) << "cycle " << cycle;
  }
}

// ---------------------------------------------------------------------------
// desiredVelocity helper
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, DesiredVelocityTrapezoidalGuard )
{
  const double v_max = 1.0, a_dec = 2.0, dt = 0.01;

  // Far away: velocity limit dominates
  EXPECT_NEAR( SafetyQpLimiter::desiredVelocity( 10.0, v_max, a_dec, dt ), v_max, 1e-12 );
  EXPECT_NEAR( SafetyQpLimiter::desiredVelocity( -10.0, v_max, a_dec, dt ), -v_max, 1e-12 );

  // Close: sqrt(2*a*d) profile dominates (braking to stop AT the target)
  const double d = 0.02;
  EXPECT_NEAR( SafetyQpLimiter::desiredVelocity( d, v_max, a_dec, dt ),
               std::sqrt( 2.0 * a_dec * d ), 1e-12 );

  // Very close: single-step reach dominates
  const double tiny = 1e-6;
  EXPECT_NEAR( SafetyQpLimiter::desiredVelocity( tiny, v_max, a_dec, dt ), tiny / dt, 1e-12 );

  // At the target: zero
  EXPECT_DOUBLE_EQ( SafetyQpLimiter::desiredVelocity( 0.0, v_max, a_dec, dt ), 0.0 );
}

// ---------------------------------------------------------------------------
// Combined scenario: flow around an obstacle and reach the target later
// ---------------------------------------------------------------------------

namespace
{
/// Simulate a 2-DoF point robot moving toward `target` past a disc obstacle.
/// Returns the number of cycles run; asserts the obstacle is never penetrated.
/// The robot stops early if it reaches the target (returns) or runs out of cycles.
int simulateDiscScenario( SafetyQpLimiter &limiter, const SafetyQpParams &params,
                          const Eigen::Vector2d &target, const Eigen::Vector2d &obstacle,
                          const double obstacle_radius, Eigen::Vector2d &q, Eigen::Vector2d &v,
                          double &min_distance_seen )
{
  const double dt = params.dt, v_max = params.v_max[0], a_dec = params.a_dec[0];
  auto in = makeInput( 2 );
  min_distance_seen = std::numeric_limits<double>::max();

  int cycles = 0;
  for ( ; cycles < 5000; ++cycles ) {
    const Eigen::Vector2d to_target = target - q;
    if ( to_target.norm() < 1e-3 && v.norm() < 1e-3 ) {
      break;
    }
    in.q = q;
    in.v_prev = v;
    for ( int i = 0; i < 2; ++i ) {
      in.v_des[i] = SafetyQpLimiter::desiredVelocity( to_target[i], v_max, a_dec, dt );
    }

    // Signed distance to the obstacle disc; gradient points away from the obstacle.
    const Eigen::Vector2d diff = q - obstacle;
    const double d = diff.norm() - obstacle_radius;
    in.collisions.clear();
    if ( d < params.d_zone ) {
      in.collisions.push_back( makeCollision( diff.normalized(), d ) );
    }

    const auto res = limiter.solve( in );
    v = res.v;
    q += v * dt;

    const double dist = ( q - obstacle ).norm() - obstacle_radius;
    min_distance_seen = std::min( min_distance_seen, dist );

    // Invariant: the ACTUAL obstacle is never penetrated. On curved obstacles the
    // linearized constraint sags below the padding by O(v^2*dt/R) per cycle — absorbing
    // that is exactly what the padding buffer is for.
    EXPECT_GT( dist, 0.0 ) << "penetrated obstacle at cycle " << cycles;
    if ( dist <= 0.0 ) {
      break;
    }
  }
  return cycles;
}
} // namespace

TEST( SafetyQpLimiter, FlowAroundGrazingObstacleReachesTarget )
{
  // The straight path to the target passes through the obstacle's safety zone (closest
  // approach 0.02 m < zone 0.05 m, but outside the disc). The damper deflects the robot
  // around the zone; it reaches the target later instead of stopping.
  auto params = makeParams( 2, /*v_max=*/1.0, /*a_acc=*/10.0, /*a_dec=*/20.0 );
  SafetyQpLimiter limiter( 2, params );

  Eigen::Vector2d q( 0.0, 0.0 ), v( 0.0, 0.0 );
  const Eigen::Vector2d target( 1.0, 0.0 );
  const Eigen::Vector2d obstacle( 0.5, 0.12 ); // 0.02 above the path with radius 0.1
  double min_dist = 0.0;

  const int cycles = simulateDiscScenario( limiter, params, target, obstacle, 0.1, q, v, min_dist );

  EXPECT_LT( ( target - q ).norm(), 1e-3 ) << "target not reached";
  // It had to deviate and slow down: strictly more cycles than the straight-line minimum
  EXPECT_GT( cycles, static_cast<int>( 1.0 / ( params.v_max[0] * params.dt ) ) );
  // The damper kept it (approximately) out of the padding the whole way
  EXPECT_GT( min_dist, 0.5 * params.d_pad );
}

TEST( SafetyQpLimiter, HeadOnObstacleStallsWithoutPenetration )
{
  // Target directly behind the obstacle: the desired velocity points straight into the
  // constraint, the tangential component vanishes → a local minimum. The damper QP is a
  // LOCAL method: expected behavior is a clean stall at the boundary (upstream replans),
  // never penetration. This pins down the behavior the controller's stall flag reports.
  auto params = makeParams( 2, /*v_max=*/1.0, /*a_acc=*/10.0, /*a_dec=*/20.0 );
  SafetyQpLimiter limiter( 2, params );

  Eigen::Vector2d q( 0.0, 0.0 ), v( 0.0, 0.0 );
  const Eigen::Vector2d target( 1.0, 0.0 );
  const Eigen::Vector2d obstacle( 0.5, 0.0 ); // dead center on the path
  double min_dist = 0.0;

  simulateDiscScenario( limiter, params, target, obstacle, 0.1, q, v, min_dist );

  EXPECT_GT( ( target - q ).norm(), 0.5 ) << "should NOT have reached the target";
  EXPECT_GT( min_dist, 0.0 );  // never penetrated
  EXPECT_LT( v.norm(), 0.05 ); // stalled (near-zero velocity)
  EXPECT_GT( q[0], 0.3 );      // but it did approach up to the boundary
}

// ---------------------------------------------------------------------------
// Held joints (hold_unrequested_joints)
// ---------------------------------------------------------------------------

TEST( SafetyQpLimiter, HeldJointIsPinnedAtZeroVelocity )
{
  // A held joint must not be recruited to satisfy the objective, even though moving it
  // is the cheapest way to reach v_des.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.8, 0.8;
  in.hold = { 0, 1 };

  const auto &r = limiter.solve( in );
  ASSERT_TRUE( r.solved );
  EXPECT_NEAR( r.v[0], 0.8, 1e-6 ) << "the free joint still tracks its demand";
  EXPECT_NEAR( r.v[1], 0.0, 1e-9 ) << "the held joint must not move";
}

TEST( SafetyQpLimiter, HeldJointCannotBePushedOutOfACollision )
{
  // The classic flipper case: joint 1 rests at its reference while joint 0 is driven
  // into it. The damper wants BOTH to separate; only joint 0 may.
  SafetyQpLimiter limiter( 2, makeParams( 2 ) );
  auto in = makeInput( 2 );
  in.v_des << 0.5, 0.0;
  in.hold = { 0, 1 };
  // Penetrating: the damper RHS is positive, i.e. it actively demands separation.
  in.collisions.push_back( makeCollision( Eigen::Vector2d( -1.0, 1.0 ), -0.005 ) );

  const auto &r = limiter.solve( in );
  EXPECT_NEAR( r.v[1], 0.0, 1e-9 ) << "the resting joint must not be swept away";
  EXPECT_LE( r.v[0], 0.0 ) << "the driven joint has to give way instead";
}

TEST( SafetyQpLimiter, HeldJointDecaysAtTheDecelerationLimit )
{
  // A joint that becomes held while moving must brake, not step to zero.
  auto params = makeParams( 1, /*v_max=*/1.0, /*a_acc=*/10.0, /*a_dec=*/20.0 );
  SafetyQpLimiter limiter( 1, params );
  auto in = makeInput( 1 );
  in.v_des << 0.0;
  in.v_prev << 1.0;
  in.hold = { 1 };

  const auto &r = limiter.solve( in );
  ASSERT_TRUE( r.solved );
  // One deceleration step from v_prev, not an instant stop.
  EXPECT_NEAR( r.v[0], 1.0 - params.a_dec[0] * params.dt, 1e-6 );
  EXPECT_LT( r.v[0], 1.0 );
}

TEST( SafetyQpLimiter, EmptyHoldVectorLeavesEveryJointFree )
{
  // Callers that do not use the mode leave `hold` empty; nothing may change.
  SafetyQpLimiter limiter( 3, makeParams( 3 ) );
  auto in = makeInput( 3 );
  in.v_des << 0.4, -0.4, 0.2;
  ASSERT_TRUE( in.hold.empty() );

  const auto &r = limiter.solve( in );
  ASSERT_TRUE( r.solved );
  EXPECT_TRUE( r.v.isApprox( in.v_des, 1e-6 ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
