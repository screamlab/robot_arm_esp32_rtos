#!/bin/bash

# Default values
topic=""
positions=""
once_flag=""
rate_flag=""

usage() {
    echo "Usage: $0 [-l|--left | -r|--right] [-o|--t-pose | -c|--rest] [--once] [--rate NUM]\n"
    echo "Options:"
    echo "   -l, --left       Use topic /left_arm."
    echo "   -r, --right      Use topic /right_arm."
    echo "   -o, --t-pose     Use T-pose joint positions."
    echo "                    For left: [3.14, 1.57, 1.57, 0.0, 1.57, 3.14]"
    echo "                    For right: [0.0, 1.57, 1.57, 0.0, 1.57, 0.0]"
    echo "   -c, --rest       Use rest joint positions."
    echo "                    For left: [3.14, 0.0, 1.57, 0.0, 1.57, 1.57]"
    echo "                    For right: [0.0, 3.14, 1.57, 0.0, 1.57, 1.57]"
    echo "   --once           Pass the --once flag to the ros2 command."
    echo "   --rate NUM       Pass --rate NUM to the ros2 command."
    exit 1
}

# If no arguments, show usage.
if [ $# -eq 0 ]; then
    usage
fi

# Parse command line options.
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -l|--left)
            topic="/left_arm"
            shift
            ;;
        -r|--right)
            topic="/right_arm"
            shift
            ;;
        -o|--t-pose)
            # Ensure topic is set
            if [ "$topic" == "/left_arm" ]; then
                positions="[3.14, 1.57, 1.57, 0.0, 1.57, 3.14]"
            elif [ "$topic" == "/right_arm" ]; then
                positions="[0.0, 1.57, 1.57, 0.0, 1.57, 0.0]"
            else
                echo "Error: Specify -l/--left or -r/--right before -c/--close."
                usage
            fi
            shift
            ;;
        -c|--rest)
            if [ "$topic" == "/left_arm" ]; then
                positions="[3.14, 0.0, 1.57, 0.0, 1.57, 1.57]"
            elif [ "$topic" == "/right_arm" ]; then
                positions="[0.0, 3.14, 1.57, 0.0, 1.57, 1.57]"
            else
                echo "Error: Specify -l/--left or -r/--right before -o/--open."
                usage
            fi
            shift
            ;;
        --once)
            once_flag="--once"
            shift
            ;;
        --rate)
            if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                rate_flag="--rate $2"
                shift 2
            else
                echo "Error: --rate requires a numeric argument."
                usage
            fi
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# Validate required options.
if [ -z "$topic" ]; then
    echo "Error: You must specify -l/--left or -r/--right."
    usage
fi

if [ -z "$positions" ]; then
    echo "Error: You must specify -c/--close or -o/--open."
    usage
fi

# Build the ros2 command.
cmd="ros2 topic pub ${once_flag} ${rate_flag} ${topic} trajectory_msgs/msg/JointTrajectoryPoint \"{positions: ${positions}, velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}\""

echo "Executing: $cmd"
# Execute the command.
eval $cmd
