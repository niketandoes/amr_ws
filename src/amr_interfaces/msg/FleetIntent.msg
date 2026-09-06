# FleetIntent.msg
# Decentralized Peer-to-Peer (P2P) Intent-Sharing Interface
# Broadcasts local robot pose, velocity, trajectory waypoints, and coordination state
# across the flat DDS multicast mesh on /fleet/intent.

# Unique identifier of the transmitting robot (e.g. 'amr1', 'amr2', 'amr3')
string robot_id

# ROS 2 timestamp of intent broadcast
builtin_interfaces/Time stamp

# Current estimated 2D/3D pose from local odometry
geometry_msgs/Pose current_pose

# Current linear and angular velocity from local odometry
geometry_msgs/Twist current_velocity

# Downsampled planned path waypoints (e.g., positions at t+1s, t+2s, t+3s)
geometry_msgs/Pose[] planned_waypoints

# Coordination state flags:
# 0 = NORMAL_NAV (default path execution)
# 1 = APPROACHING_CHOKE (within buffer of bottleneck)
# 2 = IN_CHOKE (currently occupying bottleneck reservation)
# 3 = YIELDING (holding outside bottleneck for higher-priority peer)
# 4 = BLOCKED (stalled; initiates task handoff)
uint8 current_state
