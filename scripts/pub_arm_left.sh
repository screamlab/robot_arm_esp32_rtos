#!/bin/bash

# Rest Pose
# ros2 topic pub /left_arm \
#     trajectory_msgs/msg/JointTrajectoryPoint \
#     "{positions: [0.0, 0.0, 1.57, 0.0, 1.57, 1.57], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}" --once

# T Pose
ros2 topic pub /left_arm \
    trajectory_msgs/msg/JointTrajectoryPoint \
    "{positions: [0.0, 1.57, 1.57, 0.0, 1.57, 3.14], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}" --once
