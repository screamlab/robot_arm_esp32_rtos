#!/bin/bash

# Close
# ros2 topic pub /left_hand \
#     trajectory_msgs/msg/JointTrajectoryPoint \
#     "{positions: [0.0, 3.14, 3.14, 3.14, 3.14], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}" --once

# Open
ros2 topic pub /left_hand \
    trajectory_msgs/msg/JointTrajectoryPoint \
    "{positions: [3.14, 0.0, 0.0, 0.0, 0.0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}" --once
