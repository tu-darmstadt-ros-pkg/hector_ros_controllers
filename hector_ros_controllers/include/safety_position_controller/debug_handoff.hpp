#pragma once

#include <cstdint>

#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace safety_position_controller
{

/// Hands snapshots from the control thread to a non-realtime reader without blocking:
/// a commit while the reader copies is dropped, a newer commit replaces an untaken one.
/// Committing a snapshot sized like the ones reset() prepared does not allocate.
template<typename Snapshot>
class DebugHandoff
{
public:
  /// Control thread: the snapshot to fill before commit().
  Snapshot &staging() { return staging_.data; }

  /// Control thread: hand staging() over. @return false if dropped (reader busy)
  bool commit()
  {
    ++staging_.seq;
    return box_.try_set( staging_ );
  }

  /// Reader: the snapshot committed since the last take(), or nullptr.
  const Snapshot *take()
  {
    box_.get( pending_ );
    if ( pending_.seq == taken_seq_ ) {
      return nullptr;
    }
    taken_seq_ = pending_.seq;
    return &pending_.data;
  }

  /// Clear and size both snapshots with @p prepare and drop any untaken commit.
  /// Not concurrently with commit() or take().
  template<typename Prepare>
  void reset( Prepare &&prepare )
  {
    prepare( staging_.data );
    box_.set( [&]( Entry &entry ) {
      prepare( entry.data );
      entry.seq = staging_.seq;
    } );
    taken_seq_ = staging_.seq;
  }

private:
  struct Entry {
    Snapshot data;
    uint64_t seq{ 0 }; ///< 0 = nothing committed yet
  };

  Entry staging_;
  realtime_tools::RealtimeThreadSafeBox<Entry> box_;
  Entry pending_; ///< take()'s copy
  uint64_t taken_seq_{ 0 };
};

} // namespace safety_position_controller
