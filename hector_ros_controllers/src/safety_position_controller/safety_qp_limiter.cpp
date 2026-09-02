#include "safety_position_controller/safety_qp_limiter.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace safety_position_controller
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Braking bound so that, decelerating at a_dec, motion stops after at most `dist`.
/// Discrete-time version of the sqrt(2*a*d) profile: commanding v now and stepping v
/// down by a*dt each cycle covers at most v^2/(2a) + v*dt/2, so require that <= dist:
/// v <= -a*dt/2 + sqrt(a^2*dt^2/4 + 2*a*dist). The continuous bound brakes one cycle
/// too late and overshoots the limit. dist <= 0 -> 0 (at/beyond the limit).
double positionLimitBound( const double dist, const double a_dec, const double dt )
{
  if ( dist <= 0.0 ) {
    return 0.0;
  }
  const double half_step = 0.5 * a_dec * dt;
  const double v_brake = -half_step + std::sqrt( half_step * half_step + 2.0 * a_dec * dist );
  return std::min( dist / dt, v_brake );
}
} // namespace

SafetyQpLimiter::SafetyQpLimiter( const std::size_t n, SafetyQpParams params )
    : n_( n ), params_( std::move( params ) )
{
  if ( n_ == 0 ) {
    throw std::invalid_argument( "SafetyQpLimiter: n must be > 0" );
  }
  const auto ni = static_cast<Eigen::Index>( n_ );
  if ( params_.v_max.size() != ni || params_.a_acc.size() != ni || params_.a_dec.size() != ni ) {
    throw std::invalid_argument( "SafetyQpLimiter: v_max/a_acc/a_dec must have size n" );
  }
  if ( params_.dt <= 0.0 ) {
    throw std::invalid_argument( "SafetyQpLimiter: dt must be > 0" );
  }
  if ( !( params_.d_zone > params_.d_pad ) ) {
    throw std::invalid_argument( "SafetyQpLimiter: d_zone must be > d_pad" );
  }
  if ( ( params_.v_max.array() <= 0.0 ).any() || ( params_.a_acc.array() <= 0.0 ).any() ||
       ( params_.a_dec.array() <= 0.0 ).any() ) {
    throw std::invalid_argument( "SafetyQpLimiter: v_max/a_acc/a_dec must be positive" );
  }

  // box + collision dampers
  const auto m = ni + static_cast<Eigen::Index>( params_.max_collision_constraints );

  H_ = Eigen::MatrixXd::Identity( ni, ni ) * ( 1.0 + params_.regularization );
  g_ = Eigen::VectorXd::Zero( ni );
  C_ = Eigen::MatrixXd::Zero( m, ni );
  l_ = Eigen::VectorXd::Constant( m, -kInf );
  u_ = Eigen::VectorXd::Constant( m, kInf );
  box_lb_ = Eigen::VectorXd::Zero( ni );
  box_ub_ = Eigen::VectorXd::Zero( ni );

  // Box rows are the identity block; they only change through l_/u_.
  C_.topRows( ni ) = Eigen::MatrixXd::Identity( ni, ni );

  // PrimalDualLDLT: proxsuite 0.6.5's default PrimalLDLT has an out-of-bounds access
  // in rank_r_update during mu updates (visible with Eigen assertions enabled).
  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
      ni, 0, m, /*box_constraints=*/false, proxsuite::proxqp::DenseBackend::PrimalDualLDLT,
      proxsuite::proxqp::HessianType::Dense );
  qp_->settings.eps_abs = 1e-7;
  qp_->settings.eps_rel = 0.0;
  // Healthy solves need < 50 iterations at these sizes; larger caps only get spent
  // expensively proving infeasibility.
  qp_->settings.max_iter = 200;
  qp_->settings.verbose = false;
}

SafetyQpLimiter::~SafetyQpLimiter() = default;

void SafetyQpLimiter::setCollisionDistances( const double d_pad, const double d_zone )
{
  if ( !( d_zone > d_pad ) ) {
    throw std::invalid_argument( "SafetyQpLimiter: d_zone must be > d_pad" );
  }
  params_.d_pad = d_pad;
  params_.d_zone = d_zone;
}

double SafetyQpLimiter::desiredVelocity( const double diff, const double v_max, const double a_dec,
                                         const double dt )
{
  const double mag =
      std::min( { std::abs( diff ) / dt, v_max, std::sqrt( 2.0 * a_dec * std::abs( diff ) ) } );
  return std::copysign( mag, diff );
}

void SafetyQpLimiter::brakingVelocity( const Eigen::VectorXd &v_prev, Eigen::VectorXd &v ) const
{
  v.resize( v_prev.size() );
  for ( Eigen::Index i = 0; i < v_prev.size(); ++i ) {
    const double step = params_.a_dec[i] * params_.dt;
    v[i] = v_prev[i] - std::copysign( std::min( std::abs( v_prev[i] ), step ), v_prev[i] );
  }
}

void SafetyQpLimiter::computeVelocityBounds( const SafetyQpInput &input, const bool crawl,
                                             bool &conflict )
{
  conflict = false;
  const double dt = params_.dt;

  for ( Eigen::Index i = 0; i < static_cast<Eigen::Index>( n_ ); ++i ) {
    const double v_prev = input.v_prev[i];
    const double a_acc = params_.a_acc[i];
    const double a_dec = params_.a_dec[i];

    // Acceleration box (asymmetric): moving the velocity upward is a speed-up when
    // v_prev >= 0 (accel limit) and braking when v_prev < 0 (decel limit); mirrored down.
    const double delta_up = ( v_prev >= 0.0 ? a_acc : a_dec ) * dt;
    const double delta_dn = ( v_prev > 0.0 ? a_dec : a_acc ) * dt;

    double ub = std::min( params_.v_max[i], v_prev + delta_up );
    double lb = std::max( -params_.v_max[i], v_prev - delta_dn );

    // A held joint is a crawl clamp with speed 0: the reference is not asking it to
    // move, so the QP must not recruit it to flow around or push out of a collision.
    const bool held = static_cast<std::size_t>( i ) < input.hold.size() && input.hold[i] != 0;
    if ( crawl || held ) {
      // Clamp toward +-v_clamp, decaying from the previous velocity at the deceleration
      // limit so the clamp never conflicts with the acceleration box.
      const double v_clamp = held ? 0.0 : params_.contact_crawl_speed;
      ub = std::min( ub, std::max( v_clamp, v_prev - a_dec * dt ) );
      lb = std::max( lb, std::min( -v_clamp, v_prev + a_dec * dt ) );
    }

    // Position limit braking bounds (start decelerating early enough to stop at the limit)
    if ( std::isfinite( input.q_hi[i] ) ) {
      ub = std::min( ub, positionLimitBound( input.q_hi[i] - input.q[i], a_dec, dt ) );
    }
    if ( std::isfinite( input.q_lo[i] ) ) {
      lb = std::max( lb, -positionLimitBound( input.q[i] - input.q_lo[i], a_dec, dt ) );
    }

    if ( lb > ub ) {
      if ( lb - ub <= 1e-9 ) {
        // Rounding-level disagreement (e.g. braking exactly on the position-limit
        // profile): collapse to the tighter bound instead of flagging a conflict.
        lb = ub;
      } else {
        // Mutually exclusive bounds (e.g. still too fast for the position-limit braking
        // profile). Physically the best we can do is brake as hard as allowed.
        conflict = true;
        const double brake =
            v_prev - std::copysign( std::min( std::abs( v_prev ), a_dec * dt ), v_prev );
        lb = brake;
        ub = brake;
      }
    }

    box_lb_[i] = lb;
    box_ub_[i] = ub;
  }
}

const SafetyQpResult &SafetyQpLimiter::solve( const SafetyQpInput &input )
{
  const auto t0 = std::chrono::steady_clock::now();
  const auto ni = static_cast<Eigen::Index>( n_ );

  // Reused across solves so the per-cycle vectors keep their capacity; the flags are
  // cleared by name (a new field must be reset here too).
  SafetyQpResult &result = result_;
  result.solved = false;
  result.braking = false;
  result.bounds_conflict = false;
  result.push_out_relaxed = false;
  result.num_collision_constraints = 0;
  result.solve_time_us = 0.0;
  result.iterations = 0;
  result.v.setZero( ni );

  // ---- Objective: 0.5 ||v - v_des||^2 (H is constant) ----
  g_ = -input.v_des;

  // ---- Rows [0, n): per-joint velocity box ----
  // Contact crawl: engage when any pair is inside the padding.
  bool crawl = false;
  if ( params_.contact_crawl_speed > 0.0 ) {
    for ( const auto &c : input.collisions ) {
      if ( c.distance <= params_.d_pad ) {
        crawl = true;
        break;
      }
    }
  }
  computeVelocityBounds( input, crawl, result.bounds_conflict );
  l_.head( ni ) = box_lb_;
  u_.head( ni ) = box_ub_;

  // ---- Rows [n, n+M): collision velocity dampers ----
  const auto max_cc = params_.max_collision_constraints;
  const double zone_width = params_.d_zone - params_.d_pad;
  std::size_t cc = 0;
  for ( const auto &c : input.collisions ) {
    if ( cc >= max_cc ) {
      break;
    }
    if ( c.normal.size() != ni ) {
      continue; // malformed constraint — skip rather than abort the cycle
    }
    const Eigen::Index row = ni + static_cast<Eigen::Index>( cc );
    C_.row( row ) = c.normal.transpose();
    // Damper: g^T v >= -xi * (d - d_pad) / (d_zone - d_pad); capped push-out when
    // penetrating. The approach speed is ALSO bounded by the braking envelope that can
    // still stop at the padding (the damper alone admits speeds the acceleration limits
    // cannot brake from): effective deceleration of g^T v <= sum_i |g_i| * a_dec_i.
    const double damper_rhs = -params_.damper_xi * ( c.distance - params_.d_pad ) / zone_width;
    const double a_pair = c.normal.cwiseAbs().dot( params_.a_dec );
    const double braking_rhs =
        -positionLimitBound( c.distance - params_.d_pad, std::max( a_pair, 1e-9 ), params_.dt );
    l_[row] = std::min( std::max( damper_rhs, braking_rhs ), params_.max_repulsion_speed );
    u_[row] = kInf;
    ++cc;
  }
  result.num_collision_constraints = static_cast<int>( cc );
  // Deactivate unused collision rows
  for ( std::size_t k = cc; k < max_cc; ++k ) {
    const Eigen::Index row = ni + static_cast<Eigen::Index>( k );
    C_.row( row ).setZero();
    l_[row] = -kInf;
    u_[row] = kInf;
  }

  // ---- Solve ----
  // NOTE: deliberately NOT using WARM_START_WITH_PREVIOUS_RESULT. With it, a stale
  // primal/dual pair from the previous cycle can pass proxqp's KKT residual check after
  // the bounds moved (complementarity is not re-checked), returning the OLD solution with
  // iter=0. The problems are tiny; a fresh solve is microseconds.
  using proxsuite::proxqp::InitialGuessStatus;
  qp_->settings.initial_guess = InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;
  if ( first_solve_ ) {
    qp_->init( H_, g_, std::nullopt, std::nullopt, C_, l_, u_ );
    first_solve_ = false;
  } else {
    qp_->update( std::nullopt, g_, std::nullopt, std::nullopt, C_, l_, u_ );
  }
  qp_->solve();

  auto solve_succeeded = [this]() {
    return qp_->results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED &&
           qp_->results.x.allFinite();
  };
  bool solved = solve_succeeded();
  result.iterations = static_cast<long>( qp_->results.info.iter );

  if ( !solved ) {
    // ---- Stage 2: "no-worsen" relaxation, deepest pair first ----
    // Opposing penetrating contacts make the full QP infeasible (wedged). 2a: relax all
    // push-out demands to "don't get deeper" (0) EXCEPT the deepest pair's — extract
    // from the worst contact. 2b: relax that one too; v = 0 then satisfies every row,
    // so 2b stays feasible up to transient acceleration-box conflicts, while
    // tangential/outward escape motion remains possible.
    std::size_t k_deep = cc;
    double d_deep = std::numeric_limits<double>::max();
    for ( std::size_t k = 0; k < cc; ++k ) {
      const Eigen::Index row = ni + static_cast<Eigen::Index>( k );
      if ( l_[row] > 0.0 && input.collisions[k].distance < d_deep ) {
        d_deep = input.collisions[k].distance;
        k_deep = k;
      }
    }

    bool relaxed_any = false;
    for ( std::size_t k = 0; k < cc; ++k ) {
      if ( k == k_deep ) {
        continue; // stage 2a keeps the deepest push-out demand
      }
      const Eigen::Index row = ni + static_cast<Eigen::Index>( k );
      if ( l_[row] > 0.0 ) {
        l_[row] = 0.0;
        relaxed_any = true;
      }
    }

    if ( relaxed_any ) {
      qp_->update( std::nullopt, std::nullopt, std::nullopt, std::nullopt, C_, l_, u_ );
      qp_->solve();
      solved = solve_succeeded();
      result.iterations += static_cast<long>( qp_->results.info.iter );
    }
    if ( !solved && k_deep < cc ) {
      // Stage 2b: even the deepest push-out is incompatible (true vise) — full no-worsen
      const Eigen::Index row = ni + static_cast<Eigen::Index>( k_deep );
      l_[row] = 0.0;
      qp_->update( std::nullopt, std::nullopt, std::nullopt, std::nullopt, C_, l_, u_ );
      qp_->solve();
      solved = solve_succeeded();
      result.iterations += static_cast<long>( qp_->results.info.iter );
      relaxed_any = true;
    }
    result.push_out_relaxed = relaxed_any && solved;
  }

  if ( solved ) {
    // Clamp to the box exactly: the solver satisfies constraints only up to eps_abs, and
    // the box is the hard actuator-safety part.
    result.v = qp_->results.x.cwiseMax( box_lb_ ).cwiseMin( box_ub_ );
    result.solved = true;
  } else {
    // Still infeasible (e.g. braking transient) or solver failure -> predictable
    // degradation: brake to zero at the deceleration limit.
    brakingVelocity( input.v_prev, result.v );
    result.braking = true;
  }

  // Per-constraint debug info: final RHS (after any relaxation) and achieved g^T v
  result.collision_rhs.resize( static_cast<Eigen::Index>( cc ) );
  result.collision_velocity.resize( static_cast<Eigen::Index>( cc ) );
  for ( std::size_t k = 0; k < cc; ++k ) {
    const Eigen::Index row = ni + static_cast<Eigen::Index>( k );
    result.collision_rhs[static_cast<Eigen::Index>( k )] = l_[row];
    result.collision_velocity[static_cast<Eigen::Index>( k )] = C_.row( row ).dot( result.v );
  }

  result.solve_time_us = static_cast<double>( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                  std::chrono::steady_clock::now() - t0 )
                                                  .count() ) /
                         1000.0;
  return result;
}

} // namespace safety_position_controller
