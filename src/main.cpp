#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

#include "armDriver.h"
#include "params.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// subscriber
rcl_subscription_t subscriber;
trajectory_msgs__msg__JointTrajectoryPoint msg;
rclc_executor_t executor_sub;

// publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg2;
rclc_executor_t executor_pub;
rcl_timer_t timer;

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            return false;              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do {                                   \
        static volatile int64_t init = -1; \
        if (init == -1) {                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS) {    \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

uint8_t targetAngles[NUM_OF_SERVOS];
float currentAngles[NUM_OF_SERVOS];
TaskHandle_t mainTask, armControlTask;

void subscription_callback(const void* msgin) {
    // const trajectory_msgs__msg__JointTrajectoryPoint* msg = (const trajectory_msgs__msg__JointTrajectoryPoint*)msgin;

    // Initialize the ArmManager
    // ArmManager arm_manager(NUM_OF_SERVOS, servoMinAngles, servoMaxAngles, servoInitAngles);

    // Error handling: Check if the size matches the number of servos
    // if (msg.positions.capacity != (size_t)NUM_OF_SERVOS) {
    //     return;
    // }

    // Set the target angles for each servo
    Serial.println("Recived servo angles:");
    for (size_t i = 0; i < msg.positions.capacity; ++i) {
        // arm_manager.setServoTargetAngle(i, (uint8_t)(msg->positions.data[i] * RAD_TO_DEG));
        targetAngles[i] = (uint8_t)(degrees(msg.positions.data[i]));
        Serial.print(targetAngles[i]);
    }

    // Move the arm and log the action
    // arm_manager.moveArm();
}

void publisher_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(timer);

    msg2.data = (int32_t)targetAngles[0];
    RCSOFTCHECK(rcl_publish(&publisher, &msg2, NULL));
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "right_hand_executor", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/right_arm"));

    msg.positions.capacity = NUM_OF_SERVOS;  // The number of allocated items in data
    msg.positions.size = NUM_OF_SERVOS;      // The number of valid items in data
    msg.positions.data = (double*)malloc(msg.positions.capacity * sizeof(double));
    msg.velocities.capacity = 0;
    msg.velocities.size = 0;
    msg.velocities.data = nullptr;
    msg.accelerations.capacity = 0;
    msg.accelerations.size = 0;
    msg.accelerations.data = nullptr;
    msg.effort.capacity = 0;
    msg.effort.size = 0;
    msg.effort.data = nullptr;

    // create subscriber executor
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/right_arm_2"));

    // Initialize a timer for publishing
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),  // Timer period in nanoseconds (100 ms)
        publisher_callback));

    // create publisher executor
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

    return true;
}

void destroy_entities() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // subscriber
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor_sub));

    free(msg.positions.data);
    msg.positions.data = nullptr;

    // publisher
    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor_pub));

    // common
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

void rosSubscriptionTaskFunction(void* parameter) {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

// Task function for arm control
void armControlTaskFunction(void* parameter) {
    // Create an instance of ArmManager

    // uint8_t currentAngles[ArmManager::NUM_SERVOS];
    ArmManager armManager(NUM_OF_SERVOS, servoMinAngles, servoMaxAngles, servoInitAngles);

    Serial.println("armControlTaskFunction start");

    for (;;) {
        // Control the robot arm using ArmManager
        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
            armManager.setServoTargetAngle(i, targetAngles[i]);
        }

        armManager.moveArm();
        // armManager.printStatus();
        armManager.getCurrentAngles(currentAngles);

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    state = WAITING_AGENT;

    // delay(100);
    // xTaskCreatePinnedToCore(
    //     rosSubscriptionTaskFunction,  // Task function
    //     "ROS Subscription",           // Task name
    //     2048,                         // Stack size (in bytes)
    //     NULL,                         // Task parameters
    //     1,                            // Task priority
    //     &mainTask,                    // Task handle
    //     0                             // Core ID (0 or 1)
    // );
    // delay(100);
    // xTaskCreatePinnedToCore(
    //     armControlTaskFunction,  // Task function
    //     "Arm Control Task",      // Task name
    //     2048,                    // Stack size (in bytes)
    //     NULL,                    // Task parameters
    //     1,                       // Task priority
    //     &armControlTask,         // Task handle
    //     1                        // Core ID (0 or 1)
    // );
}

void loop() {
    rosSubscriptionTaskFunction(NULL);
}
