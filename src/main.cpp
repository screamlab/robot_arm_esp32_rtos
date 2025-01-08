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

rcl_subscription_t subscriber;
trajectory_msgs__msg__JointTrajectoryPoint msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
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
    const trajectory_msgs__msg__JointTrajectoryPoint* msg = (const trajectory_msgs__msg__JointTrajectoryPoint*)msgin;

    // Initialize the ArmManager
    // ArmManager arm_manager(NUM_OF_SERVOS, servoMinAngles, servoMaxAngles, servoInitAngles);

    // Error handling: Check if the size matches the number of servos
    if (msg->positions.capacity != (size_t)NUM_OF_SERVOS) {
        return;
    }

    // Set the target angles for each servo
    for (size_t i = 0; i < NUM_OF_SERVOS; i++) {
        // arm_manager.setServoTargetAngle(i, (uint8_t)(msg->positions.data[i] * RAD_TO_DEG));
        targetAngles[i] = (uint8_t)(degrees(msg->positions.data[i]));
    }

    // Move the arm and log the action
    // arm_manager.moveArm();
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
    msg.positions.size = 0;                  // The number of valid items in data
    msg.positions.data = (double*)malloc(msg.positions.capacity * sizeof(double));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    return true;
}

void destroy_entities() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    free(msg.positions.data);
    msg.positions.data = nullptr;
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
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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
