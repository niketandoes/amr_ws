# TaskAward.msg
# Contract Net Protocol (CNP) Task Award Interface
# Broadcast by the auctioneer on /fleet/task_award announcing the auction winner
# that submitted the lowest heuristic cost.

# Unique task identifier corresponding to the concluded auction
string task_id

# Robot ID of the awarded winner who accepts and dispatches the navigation goal
string winner_id

# ROS 2 timestamp of the award publication
builtin_interfaces/Time stamp
