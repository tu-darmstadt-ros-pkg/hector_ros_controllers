#pragma once

namespace safety_position_controller
{

/**
 * @brief Stall / park state machine (pure logic, no ROS).
 *
 * stalled: the reference demands motion but the commanded motion stays ~zero for
 * stall_timeout (e.g. blocked head-on by a collision constraint).
 * parked: stalled past park_timeout — the reference is treated as abandoned; the caller
 * must zero the tracking demand until a new reference arrives (releasePark()), so a
 * blockage clearing later cannot cause unexpected delayed motion.
 *
 * Reports edge events instead of logging; the caller translates them into logs/status.
 */
class StallParkMonitor
{
public:
  struct Params {
    double stall_timeout{ 1.0 }; ///< time [s] until 'stalled' is reported
    double park_timeout{ 5.0 };  ///< time [s] until the limb parks; 0 disables parking
  };

  struct Events {
    bool stalled{ false }; ///< stall reported this cycle
    bool parked{ false };  ///< parked this cycle (caller latches the abandoned reference)
    bool resumed{ false }; ///< motion resumed after a stall (never while parked)
  };

  explicit StallParkMonitor( const Params &params ) : params_( params ) { }

  bool stalled() const { return stalled_; }
  bool parked() const { return parked_; }
  double stallTime() const { return stall_time_; }

  /// Reset stall accumulation (rebase after E-stop / state invalidation). Park survives.
  void resetStall()
  {
    stall_time_ = 0.0;
    stalled_ = false;
  }

  /// A new reference releases the parked state (and clears the stall).
  void releasePark()
  {
    parked_ = false;
    resetStall();
  }

  /// Post-solve update with this cycle's demand/motion state.
  Events update( const bool wants_motion, const bool moving, const double dt )
  {
    Events events;
    if ( wants_motion && !moving ) {
      stall_time_ += dt;
      if ( !stalled_ && stall_time_ >= params_.stall_timeout ) {
        stalled_ = true;
        events.stalled = true;
      }
      if ( !parked_ && params_.park_timeout > 0.0 && stall_time_ >= params_.park_timeout ) {
        parked_ = true;
        events.parked = true;
      }
    } else if ( !parked_ ) { // while parked, wants_motion is forced false — keep stall state
      if ( stalled_ ) {
        events.resumed = true;
      }
      resetStall();
    }
    return events;
  }

private:
  Params params_;
  double stall_time_{ 0.0 };
  bool stalled_{ false };
  bool parked_{ false };
};

} // namespace safety_position_controller
