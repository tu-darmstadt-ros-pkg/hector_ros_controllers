#include "safety_position_controller/safety_qp_limiter.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

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

/**
 * Goldfarb-Idnani dual active-set method for min 0.5 * s * ||x||^2 - c^T x subject to
 * l <= C x <= u. Starts from the unconstrained minimum and adds the most violated
 * constraint until all hold; the scaled identity Hessian makes the initial factor
 * trivial. Exact up to rounding, so the solution is independent of the previous cycle.
 * Each finite side of a row is a separate inequality: side j < m is C_j x >= l_j,
 * side j >= m is -C_{j-m} x >= -u_{j-m}.
 */
class DualActiveSetQp
{
public:
  DualActiveSetQp( const Eigen::Index n, const Eigen::Index max_rows )
      : n_( n ), max_iterations_( 10 * ( n + max_rows ) ), x_( n ), J_( n, n ), R_( n, n ),
        cx_( max_rows ), d_( n ), z_( n ), r_( n ), np_( n ), u_( n ), active_set_( n ),
        is_active_( 2 * max_rows )
  {
  }

  /// @return true if the problem with the first m rows of C/l/u was solved; false if it is
  /// infeasible or numerically degenerate.
  bool solve( const double s, const Eigen::VectorXd &c, const Eigen::MatrixXd &C,
              const Eigen::VectorXd &l, const Eigen::VectorXd &u, const Eigen::Index m )
  {
    constexpr double kFeasibilityTolerance = 1e-9;
    x_ = c / s;
    J_.setIdentity();
    J_ *= 1.0 / std::sqrt( s );
    q_ = 0;
    r_norm_ = 1.0;
    iterations_ = 0;
    is_active_.head( 2 * m ).setZero();

    while ( true ) {
      cx_.head( m ).noalias() = C.topRows( m ) * x_;
      Eigen::Index p = -1;
      double most_violated = -kFeasibilityTolerance;
      for ( Eigen::Index j = 0; j < m; ++j ) {
        if ( !is_active_[j] && cx_[j] - l[j] < most_violated ) {
          most_violated = cx_[j] - l[j];
          p = j;
        }
        if ( !is_active_[m + j] && u[j] - cx_[j] < most_violated ) {
          most_violated = u[j] - cx_[j];
          p = m + j;
        }
      }
      if ( p < 0 ) {
        return x_.allFinite();
      }
      const bool upper = p >= m;
      const Eigen::Index row = upper ? p - m : p;
      if ( upper ) {
        np_ = -C.row( row ).transpose();
      } else {
        np_ = C.row( row ).transpose();
      }
      const double b = upper ? -u[row] : l[row];

      double u_plus = 0.0;
      while ( true ) {
        if ( ++iterations_ > max_iterations_ ) {
          return false;
        }
        d_.noalias() = J_.transpose() * np_;
        z_.noalias() = J_.rightCols( n_ - q_ ) * d_.tail( n_ - q_ );
        r_.head( q_ ) = d_.head( q_ );
        R_.topLeftCorner( q_, q_ ).triangularView<Eigen::Upper>().solveInPlace( r_.head( q_ ) );

        double t_partial = kInf;
        Eigen::Index drop = -1;
        for ( Eigen::Index k = 0; k < q_; ++k ) {
          if ( r_[k] > 0.0 && u_[k] / r_[k] < t_partial ) {
            t_partial = std::max( 0.0, u_[k] / r_[k] );
            drop = k;
          }
        }
        double t_full = kInf;
        if ( z_.squaredNorm() > std::numeric_limits<double>::epsilon() ) {
          t_full = -( np_.dot( x_ ) - b ) / z_.dot( np_ );
        }
        const double t = std::min( t_partial, t_full );
        if ( !std::isfinite( t ) ) {
          return false;
        }
        if ( std::isfinite( t_full ) ) {
          x_ += t * z_;
        }
        u_.head( q_ ) -= t * r_.head( q_ );
        u_plus += t;
        if ( t == t_full ) {
          if ( !addConstraint() ) {
            return false;
          }
          active_set_[q_ - 1] = p;
          u_[q_ - 1] = u_plus;
          is_active_[p] = 1;
          break;
        }
        is_active_[active_set_[drop]] = 0;
        dropConstraint( drop );
      }
    }
  }

  const Eigen::VectorXd &x() const { return x_; }
  long iterations() const { return iterations_; }

private:
  /// Appends the constraint with d = J^T n: rotates d onto its first q+1 entries.
  bool addConstraint()
  {
    for ( Eigen::Index j = n_ - 1; j > q_; --j ) {
      double cc = d_[j - 1];
      double ss = d_[j];
      const double h = std::hypot( cc, ss );
      if ( h == 0.0 ) {
        continue;
      }
      d_[j] = 0.0;
      cc /= h;
      ss /= h;
      if ( cc < 0.0 ) {
        cc = -cc;
        ss = -ss;
        d_[j - 1] = -h;
      } else {
        d_[j - 1] = h;
      }
      const double xny = ss / ( 1.0 + cc );
      for ( Eigen::Index k = 0; k < n_; ++k ) {
        const double t1 = J_( k, j - 1 );
        const double t2 = J_( k, j );
        J_( k, j - 1 ) = t1 * cc + t2 * ss;
        J_( k, j ) = xny * ( t1 + J_( k, j - 1 ) ) - t2;
      }
    }
    R_.col( q_ ).head( q_ + 1 ) = d_.head( q_ + 1 );
    ++q_;
    if ( std::abs( d_[q_ - 1] ) <= std::numeric_limits<double>::epsilon() * r_norm_ ) {
      return false; // linearly dependent on the active set
    }
    r_norm_ = std::max( r_norm_, std::abs( d_[q_ - 1] ) );
    return true;
  }

  /// Removes active constraint k and restores the triangular R with Givens rotations.
  void dropConstraint( const Eigen::Index k )
  {
    for ( Eigen::Index i = k; i < q_ - 1; ++i ) {
      active_set_[i] = active_set_[i + 1];
      u_[i] = u_[i + 1];
      R_.col( i ) = R_.col( i + 1 );
    }
    R_.col( q_ - 1 ).head( q_ ).setZero();
    --q_;
    for ( Eigen::Index j = k; j < q_; ++j ) {
      double cc = R_( j, j );
      double ss = R_( j + 1, j );
      const double h = std::hypot( cc, ss );
      if ( h == 0.0 ) {
        continue;
      }
      cc /= h;
      ss /= h;
      R_( j + 1, j ) = 0.0;
      if ( cc < 0.0 ) {
        R_( j, j ) = -h;
        cc = -cc;
        ss = -ss;
      } else {
        R_( j, j ) = h;
      }
      const double xny = ss / ( 1.0 + cc );
      for ( Eigen::Index i = j + 1; i < q_; ++i ) {
        const double t1 = R_( j, i );
        const double t2 = R_( j + 1, i );
        R_( j, i ) = t1 * cc + t2 * ss;
        R_( j + 1, i ) = xny * ( t1 + R_( j, i ) ) - t2;
      }
      for ( Eigen::Index i = 0; i < n_; ++i ) {
        const double t1 = J_( i, j );
        const double t2 = J_( i, j + 1 );
        J_( i, j ) = t1 * cc + t2 * ss;
        J_( i, j + 1 ) = xny * ( J_( i, j ) + t1 ) - t2;
      }
    }
  }

  Eigen::Index n_;
  long max_iterations_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd J_, R_; ///< J = L^-T Q and R of the active normals' QR (L = sqrt(s) I)
  Eigen::VectorXd cx_, d_, z_, r_, np_, u_;
  Eigen::Index q_{ 0 }; ///< number of active constraints
  double r_norm_{ 1.0 };
  long iterations_{ 0 };
  Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1> active_set_;
  Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> is_active_;
};

void QpCollisionConstraints::reserve( const Eigen::Index capacity, const Eigen::Index n )
{
  normals_.resize( capacity, n );
  distances_.resize( capacity );
  size_ = 0;
}

bool QpCollisionConstraints::push( const Eigen::Ref<const Eigen::VectorXd> &normal,
                                   const double distance )
{
  if ( normal.size() != normals_.cols() ) {
    throw std::invalid_argument( "QpCollisionConstraints: normal size mismatch" );
  }
  if ( size_ == distances_.size() ) {
    return false;
  }
  normals_.row( size_ ) = normal.transpose();
  distances_[size_] = distance;
  ++size_;
  return true;
}

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

  C_ = Eigen::MatrixXd::Zero( m, ni );
  l_ = Eigen::VectorXd::Constant( m, -kInf );
  u_ = Eigen::VectorXd::Constant( m, kInf );
  box_lb_ = Eigen::VectorXd::Zero( ni );
  box_ub_ = Eigen::VectorXd::Zero( ni );
  result_.v = Eigen::VectorXd::Zero( ni );
  result_.collision_rhs = Eigen::VectorXd::Zero( m - ni );
  result_.collision_velocity = Eigen::VectorXd::Zero( m - ni );

  // Box rows are the identity block; they only change through l_/u_.
  C_.topRows( ni ) = Eigen::MatrixXd::Identity( ni, ni );

  qp_ = std::make_unique<DualActiveSetQp>( ni, m );
}

SafetyQpLimiter::~SafetyQpLimiter() = default;

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
  const QpCollisionConstraints &collisions = input.collisions;
  if ( !collisions.empty() && collisions.joints() != ni ) {
    throw std::invalid_argument( "SafetyQpLimiter: collision normals must have size n" );
  }

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

  // ---- Rows [0, n): per-joint velocity box ----
  // Contact crawl: engage when any pair is inside the padding.
  bool crawl = false;
  if ( params_.contact_crawl_speed > 0.0 ) {
    for ( Eigen::Index k = 0; k < collisions.size(); ++k ) {
      if ( collisions.distance( k ) <= params_.d_pad ) {
        crawl = true;
        break;
      }
    }
  }
  computeVelocityBounds( input, crawl, result.bounds_conflict );
  l_.head( ni ) = box_lb_;
  u_.head( ni ) = box_ub_;

  // ---- Rows [n, n+M): collision velocity dampers ----
  const Eigen::Index cc = std::min(
      collisions.size(), static_cast<Eigen::Index>( params_.max_collision_constraints ) );
  const double zone_width = params_.d_zone - params_.d_pad;
  for ( Eigen::Index k = 0; k < cc; ++k ) {
    const Eigen::Index row = ni + k;
    const double distance = collisions.distance( k );
    C_.row( row ) = collisions.normal( k );
    // Damper: g^T v >= -xi * (d - d_pad) / (d_zone - d_pad); capped push-out when
    // penetrating. The approach speed is ALSO bounded by the braking envelope that can
    // still stop at the padding (the damper alone admits speeds the acceleration limits
    // cannot brake from): effective deceleration of g^T v <= sum_i |g_i| * a_dec_i.
    const double damper_rhs = -params_.damper_xi * ( distance - params_.d_pad ) / zone_width;
    const double a_pair = C_.row( row ).cwiseAbs().dot( params_.a_dec.transpose() );
    const double braking_rhs =
        -positionLimitBound( distance - params_.d_pad, std::max( a_pair, 1e-9 ), params_.dt );
    l_[row] = std::min( std::max( damper_rhs, braking_rhs ), params_.max_repulsion_speed );
    u_[row] = kInf;
  }
  result.num_collision_constraints = static_cast<int>( cc );

  // ---- Solve: min 0.5 * (1 + regularization) * ||v||^2 - v_des^T v over the used rows ----
  const double hessian_scale = 1.0 + params_.regularization;
  const Eigen::Index rows = ni + cc;
  bool solved = qp_->solve( hessian_scale, input.v_des, C_, l_, u_, rows );
  result.iterations = qp_->iterations();

  if ( !solved ) {
    // ---- Stage 2: "no-worsen" relaxation, deepest pair first ----
    // Opposing penetrating contacts make the full QP infeasible (wedged). 2a: relax all
    // push-out demands to "don't get deeper" (0) EXCEPT the deepest pair's — extract
    // from the worst contact. 2b: relax that one too; v = 0 then satisfies every row,
    // so 2b stays feasible up to transient acceleration-box conflicts, while
    // tangential/outward escape motion remains possible.
    Eigen::Index k_deep = cc;
    double d_deep = std::numeric_limits<double>::max();
    for ( Eigen::Index k = 0; k < cc; ++k ) {
      if ( l_[ni + k] > 0.0 && collisions.distance( k ) < d_deep ) {
        d_deep = collisions.distance( k );
        k_deep = k;
      }
    }

    bool relaxed_any = false;
    for ( Eigen::Index k = 0; k < cc; ++k ) {
      if ( k == k_deep ) {
        continue; // stage 2a keeps the deepest push-out demand
      }
      const Eigen::Index row = ni + k;
      if ( l_[row] > 0.0 ) {
        l_[row] = 0.0;
        relaxed_any = true;
      }
    }

    if ( relaxed_any ) {
      solved = qp_->solve( hessian_scale, input.v_des, C_, l_, u_, rows );
      result.iterations += qp_->iterations();
    }
    if ( !solved && k_deep < cc ) {
      // Stage 2b: even the deepest push-out is incompatible (true vise) — full no-worsen
      l_[ni + k_deep] = 0.0;
      solved = qp_->solve( hessian_scale, input.v_des, C_, l_, u_, rows );
      result.iterations += qp_->iterations();
      relaxed_any = true;
    }
    result.push_out_relaxed = relaxed_any && solved;
  }

  if ( solved ) {
    // Clamp to the box exactly: the solver satisfies constraints only up to its
    // feasibility tolerance, and the box is the hard actuator-safety part.
    result.v = qp_->x().cwiseMax( box_lb_ ).cwiseMin( box_ub_ );
    result.solved = true;
  } else {
    // Still infeasible (e.g. braking transient) or solver failure -> predictable
    // degradation: brake to zero at the deceleration limit.
    brakingVelocity( input.v_prev, result.v );
    result.braking = true;
  }

  // Per-constraint debug info: final RHS (after any relaxation) and achieved g^T v
  result.collision_rhs.head( cc ) = l_.segment( ni, cc );
  result.collision_velocity.head( cc ).noalias() = C_.middleRows( ni, cc ) * result.v;

  result.solve_time_us = static_cast<double>( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                  std::chrono::steady_clock::now() - t0 )
                                                  .count() ) /
                         1000.0;
  return result;
}

} // namespace safety_position_controller
