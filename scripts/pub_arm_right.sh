#!/bin/bash

# Rest Pose
# ros2 topic pub /right_arm \
#     trajectory_msgs/msg/JointTrajectoryPoint \
#     "{positions: [0.0, 3.14, 1.57, 0.0, 1.57, 1.57], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}" --once

# T Pose
ros2 topic pub /right_arm \
    trajectory_msgs/msg/JointTrajectoryPoint \
    "{positions: [0.0, 1.57, 1.57, 0.0, 1.57, 0.0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}" --once
