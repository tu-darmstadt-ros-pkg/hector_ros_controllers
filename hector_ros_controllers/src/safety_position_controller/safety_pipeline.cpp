#include "safety_position_controller/safety_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace safety_position_controller
{

SafetyPipeline::SafetyPipeline( Config config )
    : config_( std::move( config ) ), monitor_( config_.stall_park )
{
  const std::size_t n = config_.joints.size();
  const auto ni = static_cast<Eigen::Index>( n );
  if ( n == 0 ) {
    throw std::invalid_argument( "SafetyPipeline: joints must not be empty" );
  }
  if ( config_.deviation_limits.size() != n ) {
    throw std::invalid_argument( "SafetyPipeline: deviation_limits size mismatch" );
  }
  if ( config_.joint_v_index.empty() ) {
    config_.joint_v_index.assign( n, -1 );
  } else if ( config_.joint_v_index.size() != n ) {
    throw std::invalid_argument( "SafetyPipeline: joint_v_index size mismatch" );
  }

  limiter_ = std::make_unique<SafetyQpLimiter>( n, config_.qp ); // validates qp params

  cmd_ = Eigen::VectorXd::Zero( ni );
  vel_ = Eigen::VectorXd::Zero( ni );
  ref_leashed_ = Eigen::VectorXd::Zero( ni );
  input_.v_des = Eigen::VectorXd::Zero( ni );
  input_.v_prev = Eigen::VectorXd::Zero( ni );
  input_.q = Eigen::VectorXd::Zero( ni );
  input_.q_lo = Eigen::VectorXd::Constant( ni, -std::numeric_limits<double>::infinity() );
  input_.q_hi = Eigen::VectorXd::Constant( ni, std::numeric_limits<double>::infinity() );
  input_.collisions.reserve( config_.qp.max_collision_constraints );
  constraint_pair_indices_.reserve( config_.qp.max_collision_constraints );
  reference_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  measured_.assign( n, std::numeric_limits<double>::quiet_NaN() );
  parked_reference_.assign( n, std::numeric_limits<double>::quiet_NaN() );
}

bool SafetyPipeline::prepare( const std::vector<double> &reference,
                              const std::vector<double> &measured, const bool bypass_active )
{
  const std::size_t n = config_.joints.size();
  reference_.assign( reference.begin(), reference.end() );
  measured_.assign( measured.begin(), measured.end() );

  if ( !state_valid_ ) {
    for ( std::size_t i = 0; i < n; ++i ) { cmd_[static_cast<Eigen::Index>( i )] = measured[i]; }
    vel_.setZero();
    monitor_.resetStall();
    state_valid_ = true;
  }
  if ( park_pending_ ) {
    monitor_.park();
    parked_reference_ = reference_;
    park_pending_ = false;
  }

  // ---- Desired velocity toward the (leashed) reference ----
  wants_motion_ = false;
  for ( std::size_t i = 0; i < n; ++i ) {
    const auto idx = static_cast<Eigen::Index>( i );
    const JointInfo &joint = config_.joints[i];
    const double target = reference[i];
    double diff = 0.0;
    if ( !std::isnan( target ) ) {
      diff = ( joint.type == JointType::CONTINUOUS ) ? get_signed_distance( cmd_[idx], target )
                                                     : ( target - cmd_[idx] );
      if ( config_.reference_leash_time > 0.0 ) {
        const double leash = config_.qp.v_max[idx] * config_.reference_leash_time;
        diff = std::clamp( diff, -leash, leash );
      }
    }
    ref_leashed_[idx] = cmd_[idx] + diff;
    input_.v_des[idx] = SafetyQpLimiter::desiredVelocity( diff, config_.qp.v_max[idx],
                                                          config_.qp.a_dec[idx], config_.dt );
    wants_motion_ |= std::abs( input_.v_des[idx] ) > config_.stall_velocity_threshold;

    // Position limits (damper handled inside the QP); bypass extends them like the
    // reference clamp does
    if ( joint.has_position_limits && joint.type != JointType::CONTINUOUS ) {
      const double tolerance =
          bypass_active ? ( joint.upper_limit - joint.lower_limit ) * config_.bypass_limit_tolerance
                        : 0.0;
      input_.q_lo[idx] = joint.lower_limit - tolerance;
      input_.q_hi[idx] = joint.upper_limit + tolerance;
    } else {
      input_.q_lo[idx] = -std::numeric_limits<double>::infinity();
      input_.q_hi[idx] = std::numeric_limits<double>::infinity();
    }

    // Per-joint deviation box around the leashed reference: bounds how far every link
    // may leave the upstream-validated path. One-sided (widened to include the current
    // command): prevents drifting further, never demands catch-up.
    const double dev_limit = bypass_active ? 0.0 : config_.deviation_limits[i];
    if ( dev_limit > 0.0 ) {
      input_.q_lo[idx] =
          std::max( input_.q_lo[idx], std::min( ref_leashed_[idx] - dev_limit, cmd_[idx] ) );
      input_.q_hi[idx] =
          std::min( input_.q_hi[idx], std::max( ref_leashed_[idx] + dev_limit, cmd_[idx] ) );
    }
  }

  // ---- Parked: the latched reference is abandoned; hold until a NEW command ----
  bool resumed_from_park = false;
  if ( monitor_.parked() ) {
    if ( isNewReference( reference ) ) {
      monitor_.releasePark();
      resumed_from_park = true;
    } else {
      input_.v_des.setZero();
      wants_motion_ = false;
    }
  }

  input_.v_prev = vel_;
  input_.q = cmd_;
  return resumed_from_park;
}

SafetyPipeline::Events SafetyPipeline::step( const CollisionObservation &obs )
{
  const std::size_t n = config_.joints.size();

  // ---- Collision damper constraints ----
  input_.collisions.clear();
  constraint_pair_indices_.clear();
  if ( obs.checks_active ) {
    if ( obs.state_valid && obs.pairs ) {
      for ( const auto &candidate : *obs.pairs ) {
        QpCollisionConstraint c;
        c.distance = candidate.distance;
        c.normal.resize( static_cast<Eigen::Index>( n ) );
        for ( std::size_t i = 0; i < n; ++i ) {
          const int v_idx = config_.joint_v_index[i];
          c.normal[static_cast<Eigen::Index>( i )] =
              ( candidate.gradient && v_idx >= 0 && v_idx < candidate.gradient->size() )
                  ? ( *candidate.gradient )[v_idx]
                  : 0.0;
        }
        // A pair the controlled joints cannot influence must not constrain (or even
        // infeasible-block) the QP.
        if ( c.normal.norm() > 1e-12 ) {
          input_.collisions.push_back( std::move( c ) );
          constraint_pair_indices_.push_back( candidate.pair_index );
        }
      }
      if ( obs.in_collision && input_.collisions.empty() ) {
        // In collision but no usable constraints (e.g. NaN/Inf positions → blanket
        // collision without pairs): state untrusted → stop demanding motion, QP brakes.
        input_.v_des.setZero();
        wants_motion_ = false;
      }
    } else {
      // Safety state unobservable → stop demanding motion; the QP brakes smoothly.
      input_.v_des.setZero();
      wants_motion_ = false;
    }
  }

  // ---- Solve, integrate, clamp ----
  result_ = limiter_->solve( input_ );
  vel_ = result_.v;
  cmd_ += vel_ * config_.dt;

  if ( config_.tracking_leash > 0.0 ) {
    // Anti-windup: never run further ahead of the measured position than the leash.
    // Continuous joints integrate cmd_ freely while the hardware may report wrapped
    // angles, so the lag is the shortest angular distance and the correction is applied
    // to cmd_ instead of rebasing it onto the measured frame.
    for ( std::size_t i = 0; i < n; ++i ) {
      const auto idx = static_cast<Eigen::Index>( i );
      const double ahead = ( config_.joints[i].type == JointType::CONTINUOUS )
                               ? get_signed_distance( measured_[i], cmd_[idx] )
                               : ( cmd_[idx] - measured_[i] );
      if ( std::abs( ahead ) > config_.tracking_leash ) {
        cmd_[idx] += std::copysign( config_.tracking_leash, ahead ) - ahead;
      }
    }
  }

  // ---- Stall detection: reference demands motion but the QP output is ~zero ----
  Events events;
  const bool moving = vel_.cwiseAbs().maxCoeff() > config_.stall_velocity_threshold;
  events.stall = monitor_.update( wants_motion_, moving, config_.dt );
  if ( events.stall.parked ) {
    parked_reference_ = reference_;
  }
  return events;
}

bool SafetyPipeline::isNewReference( const std::vector<double> &reference ) const
{
  for ( std::size_t i = 0; i < config_.joints.size(); ++i ) {
    const double ref = reference[i];
    if ( std::isnan( ref ) ) {
      continue;
    }
    if ( std::isnan( parked_reference_[i] ) ) {
      return true;
    }
    const double delta = ( config_.joints[i].type == JointType::CONTINUOUS )
                             ? get_signed_distance( parked_reference_[i], ref )
                             : ( ref - parked_reference_[i] );
    if ( std::abs( delta ) > config_.park_resume_threshold ) {
      return true;
    }
  }
  return false;
}

} // namespace safety_position_controller
