#pragma once

#include <cstdint>

#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace safety_position_controller
{

/**
 * @brief Hands debug snapshots from the control thread to a non-realtime reader.
 *
 * The control thread fills staging() and commit()s it; take() returns each committed
 * snapshot at most once. A commit while the reader is copying is dropped instead of
 * waiting, and a newer commit overwrites one not yet taken. Once reset() has sized the
 * snapshots, a commit of an equally sized snapshot does not allocate.
 */
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

  /// Reader: the snapshot committed since the last take(), or nullptr. Valid until the
  /// next take() or reset().
  const Snapshot *take()
  {
    box_.get( pending_ );
    if ( pending_.seq == taken_seq_ ) {
      return nullptr;
    }
    taken_seq_ = pending_.seq;
    return &pending_.data;
  }

  /// Apply @p prepare to the staging and the handed-over snapshot (to clear and size
  /// them) and drop any commit not taken yet. Not concurrently with commit() or take().
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
