# TaskBid.msg
# Contract Net Protocol (CNP) Task Bid Interface
# Submitted by candidate peers on /fleet/task_bids offering their heuristic cost
# based on travel distance, battery state of charge (SOC), and availability.

# Task identifier matching the corresponding TaskAuction
string task_id

# Robot ID of the bidding agent (e.g., 'amr2', 'amr3')
string bidder_id

# Heuristic bid cost calculated locally: Cost = (1.0 * Distance) + (0.5 * (100 - Battery_SOC))
float32 bid_cost

# ROS 2 timestamp of the bid submission
builtin_interfaces/Time stamp
