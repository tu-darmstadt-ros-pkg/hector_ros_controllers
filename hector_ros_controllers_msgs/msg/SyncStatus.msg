std_msgs/Header header

# Joint names
string[] joint_names

# Per-joint input velocity command (from reference interface)
float64[] vel_command_in

# Per-joint output velocity command (input + sync correction, clamped)
float64[] vel_command_out

# Per-joint desired sync offset to first sync partner (NaN if not in a sync group)
float64[] desired_sync_offset

# Per-joint current sync offset to first sync partner (NaN if not in a sync group)
float64[] current_sync_offset
