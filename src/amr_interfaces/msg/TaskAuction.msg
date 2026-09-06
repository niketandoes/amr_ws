# TaskAuction.msg
# Contract Net Protocol (CNP) Task Auction Interface
# Broadcast by a blocked or overloaded AMR on /fleet/task_auction to initiate
# a decentralized reverse auction for task reassignment.

# Unique task identifier (e.g., 'task_amr1_84920')
string task_id

# Robot ID of the auctioneer issuing the task handoff request
string auctioneer_id

# Target destination coordinates to be fulfilled by the winning bidder
geometry_msgs/Pose target_pose

# Auction duration window in seconds (default 0.5s) before awarding lowest bidder
float32 timeout_seconds

# ROS 2 timestamp of the auction announcement
builtin_interfaces/Time stamp
