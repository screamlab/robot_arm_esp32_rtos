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



### Mutex Lock

The `#define USE_MUTEX_LOCK` directive in `include/params.hpp` controls the code to use the mutex lock.

To disable the mutex lock, please comment out this line.



### UART2 Logging (For Debug, Recommend)

The `#define USE_UART2` directive in `include/params.hpp` controls the code to log the joint data via UART2. We use Rx2 and Tx2, defined as GPIO16 and GPIO17, to transmit joint data.

Connect the Rx (white wire) of the USB-to-UART converter to the Tx2 (GPIO17) on the ESP32, and connect the GND (black wire) to the ESP32 ground.

Finally, execute the following command:

```bash
screen /dev/ttyUSB0 921600
```

To exit the `screen`, press `Ctrl-A`, then `K`, and confirm.

![ESP32 Pin](https://www.upesy.com/cdn/shop/files/doc-esp32-pinout-reference-wroom-devkit.png?width=1038)



### Re-Publish Data (For Debug, NOT Recommend)

The `#define USE_REPUBLISH` directive in `include/params.hpp` controls the code to republish the joint data. We subscribe the joint angles from `/NAMESPACE/TOPIC_NAME`, convert the radian values to degrees, and then publish them to `/NAMESPACE/REPUBLISH_TOPIC_NAME`.

This will increase the overall workload on the middleware.



[Example](https://github.com/botamochi6277/micro_ros_arduino_pub_sub_example/blob/main/src/main.cpp) for publishing and subscribing data.



## Testing

We provide several simple scripts to publish joint angles to `ARM_TOPIC_NAME` and `HAND_TOPIC_NAME` to test your robot arm.

