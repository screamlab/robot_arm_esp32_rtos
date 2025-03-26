# ESP32 C++ Code for Controlling Robot Arm

We need the following items.

- Visual Studio Code
- PlatformIO



## Environment Settings

### Windows Driver for ESP32

1. Download the [CP210x Universal Windows Driver](https://www.silabs.com/documents/public/software/CP210x_Universal_Windows_Driver.zip).
2. Unzip the compressed file.
3. Right-click the `silabser.inf` and select "install".



## On-Board LED State for ESP32 Dev. Board

- Waiting to connect to the Micro ROS agent: The LED flashes.

- Successfully connected to the Micro ROS agent: The LED remains steady.



## Parameters

### Left Arm or Right Arm

The `#define IS_RIGHT` directive in `include/params.hpp` controls the parameter setting to right arm.

To use the left arm, please comment out this line.



### Shared Memory

The `double joint_positions[NUM_ALL_SERVOS]` array, defined in `src/main.cpp`, serves as shared memory. Micro ROS writes into the shared memory based on subscribed data, while the arm controller reads from it to execute movements of the robot arm.



## Debugging

We subscribe the joint angles from `/NAMESPACE/ARM_TOPIC_NAME`, convert the radian values to degrees, and then publish them to `/NAMESPACE/ARM_REPUBLISH_TOPIC_NAME`. Similarly, we subscribe to `/NAMESPACE/HAND_TOPIC_NAME` and publish to `/NAMESPACE/HAND_REPUBLISH_TOPIC_NAME`.



[Example](https://github.com/botamochi6277/micro_ros_arduino_pub_sub_example/blob/main/src/main.cpp) for publishing and subscribing data.



