#ifndef SYNC_GROUP_VELOCITY_TO_POSITION_CONTROLLER__SYNC_PAIR_MANAGER_HPP_
#define SYNC_GROUP_VELOCITY_TO_POSITION_CONTROLLER__SYNC_PAIR_MANAGER_HPP_

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace sync_group_velocity_to_position_controller
{

/**
 * \brief Manages symmetric sync offsets for joint pairs.
 *
 * Stores one canonical offset per pair (pos_B - pos_A, where A < B by joint index).
 * Each joint sees the offset from its own perspective via a sign factor.
 * This guarantees that the offset seen by joint A is always the exact negative
 * of the offset seen by joint B — enforced by construction.
 *
 * Assumes each group contains exactly 2 joints and no joint belongs to more than one group.
 */
class SyncPairManager
{
public:
  static constexpr size_t NO_PARTNER = std::numeric_limits<size_t>::max();

  SyncPairManager() = default;

  /**
   * \brief Build pair topology from a groups map.
   * \param num_joints  Total number of joints
   * \param groups      Map from group name to vector of joint indices (each must have exactly 2)
   */
  inline void init( size_t num_joints,
                    const std::unordered_map<std::string, std::vector<size_t>> &groups )
  {
    joint_links_.assign( num_joints, { NO_PARTNER, 0, 0.0 } );
    pair_offsets_.clear();

    for ( const auto &group : groups ) {
      const auto &members = group.second;
      if ( members.size() != 2 )
        continue;

      size_t a = std::min( members[0], members[1] );
      size_t b = std::max( members[0], members[1] );
      size_t pair_idx = pair_offsets_.size();

      pair_offsets_.push_back( std::numeric_limits<double>::quiet_NaN() );

      // joint_a (smaller index): sign = +1 → get_offset returns +(pos_B - pos_A)
      // joint_b (larger index):  sign = -1 → get_offset returns -(pos_B - pos_A) = pos_A - pos_B
      joint_links_[a] = { b, pair_idx, +1.0 };
      joint_links_[b] = { a, pair_idx, -1.0 };
    }
  }

  /// Returns true if the joint has a sync partner.
  inline bool has_partner( size_t joint_idx ) const
  {
    return joint_links_[joint_idx].partner != NO_PARTNER;
  }

  /// Returns the partner joint index. Precondition: has_partner(joint_idx) is true.
  inline size_t partner( size_t joint_idx ) const { return joint_links_[joint_idx].partner; }

  /**
   * \brief Get the offset from this joint's perspective.
   * \return pos[partner] - pos[self] at the time the offset was recorded.
   */
  inline double get_offset( size_t joint_idx ) const
  {
    const auto &link = joint_links_[joint_idx];
    return link.sign * pair_offsets_[link.pair_index];
  }

  /// Check if the offset for this joint's pair is NaN (uninitialized).
  inline bool is_offset_nan( size_t joint_idx ) const
  {
    return std::isnan( pair_offsets_[joint_links_[joint_idx].pair_index] );
  }

  /**
   * \brief Set the offset from this joint's perspective.
   * \param joint_idx  The joint setting the offset
   * \param value      The offset as pos[partner] - pos[joint_idx]
   *
   * Internally converts to canonical form (pos_B - pos_A) using the sign factor.
   */
  inline void set_offset( size_t joint_idx, double value )
  {
    const auto &link = joint_links_[joint_idx];
    pair_offsets_[link.pair_index] = link.sign * value;
  }

  /// Reset all pair offsets to NaN.
  inline void reset_all()
  {
    for ( auto &offset : pair_offsets_ ) { offset = std::numeric_limits<double>::quiet_NaN(); }
  }

private:
  struct JointLink {
    size_t partner;    ///< Partner joint index, or NO_PARTNER if not in a pair
    size_t pair_index; ///< Index into pair_offsets_
    double sign;       ///< +1.0 if this joint is the smaller index, -1.0 otherwise
  };

  std::vector<JointLink> joint_links_;
  std::vector<double> pair_offsets_;
};

} // namespace sync_group_velocity_to_position_controller

#endif // SYNC_GROUP_VELOCITY_TO_POSITION_CONTROLLER__SYNC_PAIR_MANAGER_HPP_
