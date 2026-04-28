#ifndef SYNC_GROUP_VELOCITY_TO_POSITION_CONTROLLER__TRAPEZOIDAL_PROFILE_HPP_
#define SYNC_GROUP_VELOCITY_TO_POSITION_CONTROLLER__TRAPEZOIDAL_PROFILE_HPP_

#include <cmath>
#include <utility>

namespace sync_group_velocity_to_position_controller
{

/// RT-safe trapezoidal velocity profile (pure math, no allocations).
/// Generates a position trajectory from start to target with bounded velocity and acceleration.
/// Falls back to a triangular profile if the distance is too short for full cruise.
struct TrapezoidalProfile {
  double start_position = 0.0;
  double target_position = 0.0;
  double max_velocity = 0.0; ///< unsigned magnitude
  double acceleration = 0.0; ///< unsigned magnitude
  double t_accel = 0.0;      ///< acceleration phase duration
  double t_cruise = 0.0;     ///< constant-velocity phase duration
  double t_decel = 0.0;      ///< deceleration phase duration
  double total_time = 0.0;
  int direction = 1; ///< +1 or -1

  /// Compute a trapezoidal profile from start to target.
  static TrapezoidalProfile compute( double start, double target, double max_vel, double max_accel )
  {
    TrapezoidalProfile p;
    p.start_position = start;
    p.target_position = target;
    p.max_velocity = std::abs( max_vel );
    p.acceleration = std::abs( max_accel );

    const double distance = target - start;
    p.direction = ( distance >= 0.0 ) ? 1 : -1;
    const double abs_distance = std::abs( distance );

    if ( abs_distance < 1e-9 || p.acceleration < 1e-9 || p.max_velocity < 1e-9 ) {
      // Already at target, no acceleration, or no velocity configured
      p.total_time = 0.0;
      return p;
    }

    // Distance needed to accelerate to max_vel and decelerate back to zero
    const double dist_for_full_trapezoid = ( p.max_velocity * p.max_velocity ) / p.acceleration;

    if ( abs_distance >= dist_for_full_trapezoid ) {
      // Full trapezoidal profile
      p.t_accel = p.max_velocity / p.acceleration;
      p.t_decel = p.t_accel;
      const double cruise_distance = abs_distance - dist_for_full_trapezoid;
      p.t_cruise = cruise_distance / p.max_velocity;
    } else {
      // Triangular profile: cannot reach max velocity
      // v_peak = sqrt(abs_distance * acceleration)
      const double v_peak = std::sqrt( abs_distance * p.acceleration );
      p.t_accel = v_peak / p.acceleration;
      p.t_decel = p.t_accel;
      p.t_cruise = 0.0;
      // Override max_velocity to the achievable peak for evaluate()
      p.max_velocity = v_peak;
    }

    p.total_time = p.t_accel + p.t_cruise + p.t_decel;
    return p;
  }

  /// Compute a braking (deceleration-only) profile from current position/velocity to a stop.
  /// The joint decelerates from initial_velocity to zero at the given deceleration rate.
  static TrapezoidalProfile compute_braking( double start, double initial_velocity,
                                             double deceleration )
  {
    TrapezoidalProfile p;
    p.start_position = start;
    p.acceleration = std::abs( deceleration );
    p.max_velocity = std::abs( initial_velocity );
    p.direction = ( initial_velocity >= 0.0 ) ? 1 : -1;

    if ( p.max_velocity < 1e-9 || p.acceleration < 1e-9 ) {
      p.target_position = start;
      p.total_time = 0.0;
      return p;
    }

    // Deceleration only: time = v/a, distance = v^2/(2*a)
    p.t_accel = 0.0;
    p.t_cruise = 0.0;
    p.t_decel = p.max_velocity / p.acceleration;
    p.total_time = p.t_decel;

    const double braking_distance = ( p.max_velocity * p.max_velocity ) / ( 2.0 * p.acceleration );
    p.target_position = start + static_cast<double>( p.direction ) * braking_distance;

    return p;
  }

  /// Evaluate position and velocity at time t. RT-safe (pure math).
  /// Returns (position, velocity).
  std::pair<double, double> evaluate( double t ) const
  {
    if ( t <= 0.0 ) {
      return { start_position, 0.0 };
    }
    if ( t >= total_time ) {
      return { target_position, 0.0 };
    }

    const double dir = static_cast<double>( direction );

    if ( t < t_accel ) {
      // Acceleration phase
      const double vel = acceleration * t;
      const double pos = start_position + dir * 0.5 * acceleration * t * t;
      return { pos, dir * vel };
    }

    // Position at end of acceleration phase
    const double pos_after_accel = start_position + dir * 0.5 * acceleration * t_accel * t_accel;

    if ( t < t_accel + t_cruise ) {
      // Cruise phase
      const double dt_cruise = t - t_accel;
      const double pos = pos_after_accel + dir * max_velocity * dt_cruise;
      return { pos, dir * max_velocity };
    }

    // Position at end of cruise phase
    const double pos_after_cruise = pos_after_accel + dir * max_velocity * t_cruise;

    // Deceleration phase
    const double dt_decel = t - t_accel - t_cruise;
    const double vel = max_velocity - acceleration * dt_decel;
    const double pos = pos_after_cruise +
                       dir * ( max_velocity * dt_decel - 0.5 * acceleration * dt_decel * dt_decel );
    return { pos, dir * vel };
  }
};

} // namespace sync_group_velocity_to_position_controller

#endif // SYNC_GROUP_VELOCITY_TO_POSITION_CONTROLLER__TRAPEZOIDAL_PROFILE_HPP_
