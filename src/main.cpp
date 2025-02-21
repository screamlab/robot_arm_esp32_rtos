#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

#include <armDriver.hpp>
#include <esp32_led.hpp>
#include <params.hpp>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// subscriber
rcl_subscription_t arm_sub, hand_sub;
trajectory_msgs__msg__JointTrajectoryPoint arm_msg_sub, hand_msg_sub;
rclc_executor_t arm_executor_sub, hand_executor_sub;

// publisher
rcl_publisher_t arm_pub, hand_pub;
trajectory_msgs__msg__JointTrajectoryPoint arm_msg_pub, hand_msg_pub;
rclc_executor_t arm_executor_pub, hand_executor_pub;
rcl_timer_t arm_timer, hand_timer;

// Global variables shared between the microROS task and the arm control task
double joint_positions[NUM_ALL_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

states state;

void arm_subscription_callback(const void *msgin) {
    const trajectory_msgs__msg__JointTrajectoryPoint *msg = (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;
    for (size_t i = 0; i < msg->positions.size; ++i) {
        joint_positions[i + ARM_OFFSET] = degrees(msg->positions.data[i]);
    }
}

void arm_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&arm_pub, &arm_msg_pub, NULL));
        for (size_t i = 0; i < arm_msg_pub.positions.capacity; i++) {
            arm_msg_pub.positions.data[i] = joint_positions[i + ARM_OFFSET];
        }
    }
}

void hand_subscription_callback(const void *msgin) {
    const trajectory_msgs__msg__JointTrajectoryPoint *msg = (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;
    for (size_t i = 0; i < msg->positions.size; ++i) {
        joint_positions[i + HAND_OFFSET] = degrees(msg->positions.data[i]);
    }
}

void hand_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&hand_pub, &hand_msg_pub, NULL));
        for (size_t i = 0; i < hand_msg_pub.positions.capacity; i++) {
            hand_msg_pub.positions.data[i] = joint_positions[i + HAND_OFFSET];
        }
    }
}

/**
 * Functions create_entities and destroy_entities can take several seconds.
 * In order to reduce this rebuild the library with
 * - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
 * - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
 */
bool create_entities() {
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create arm subscriber
    RCCHECK(rclc_subscription_init_default(
        &arm_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/right_arm"));

    arm_msg_sub.positions.capacity = NUM_ARM_SERVOS;
    arm_msg_sub.positions.size = NUM_ARM_SERVOS;
    arm_msg_sub.positions.data = (double *)calloc(arm_msg_sub.positions.capacity, sizeof(double));

    // create arm subscriber executor
    RCCHECK(rclc_executor_init(&arm_executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&arm_executor_sub, &arm_sub, &arm_msg_sub, &arm_subscription_callback, ON_NEW_DATA));

    // create hand subscriber
    RCCHECK(rclc_subscription_init_default(
        &hand_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/right_hand"));

    hand_msg_sub.positions.capacity = NUM_HAND_SERVOS;
    hand_msg_sub.positions.size = NUM_HAND_SERVOS;
    hand_msg_sub.positions.data = (double *)calloc(hand_msg_sub.positions.capacity, sizeof(double));

    // create hand subscriber executor
    RCCHECK(rclc_executor_init(&hand_executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&hand_executor_sub, &hand_sub, &hand_msg_sub, &hand_subscription_callback, ON_NEW_DATA));

    // create arm publisher
    RCCHECK(rclc_publisher_init_default(
        &arm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/right_arm_republish"));

    // create arm timer, this timer sets the period for publishing data.
    const unsigned int timer_timeout = 50;
    RCCHECK(rclc_timer_init_default(
        &arm_timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),  // Timer period in nanoseconds
        arm_timer_callback));

    arm_msg_pub.positions.capacity = NUM_ARM_SERVOS;
    arm_msg_pub.positions.size = NUM_ARM_SERVOS;
    arm_msg_pub.positions.data = (double *)calloc(arm_msg_pub.positions.capacity, sizeof(double));

    // create arm publisher executor
    RCCHECK(rclc_executor_init(&arm_executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&arm_executor_pub, &arm_timer));

    // create hand publisher
    RCCHECK(rclc_publisher_init_default(
        &hand_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/right_hand_republish"));

    // create hand timer, this timer sets the period for publishing data.
    RCCHECK(rclc_timer_init_default(
        &hand_timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),  // Timer period in nanoseconds
        hand_timer_callback));

    hand_msg_pub.positions.capacity = NUM_HAND_SERVOS;
    hand_msg_pub.positions.size = NUM_HAND_SERVOS;
    hand_msg_pub.positions.data = (double *)calloc(hand_msg_pub.positions.capacity, sizeof(double));

    // create hand publisher executor
    RCCHECK(rclc_executor_init(&hand_executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&hand_executor_pub, &hand_timer));

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // arm subscriber
    RCSOFTCHECK(rcl_subscription_fini(&arm_sub, &node));
    RCSOFTCHECK(rclc_executor_fini(&arm_executor_sub));

    free(arm_msg_sub.positions.data);
    arm_msg_sub.positions.data = NULL;

    // hand subscriber
    RCSOFTCHECK(rcl_subscription_fini(&hand_sub, &node));
    RCSOFTCHECK(rclc_executor_fini(&hand_executor_sub));

    free(hand_msg_sub.positions.data);
    hand_msg_sub.positions.data = NULL;

    // arm publisher
    RCSOFTCHECK(rcl_publisher_fini(&arm_pub, &node));
    RCSOFTCHECK(rcl_timer_fini(&arm_timer));
    RCSOFTCHECK(rclc_executor_fini(&arm_executor_pub));

    free(arm_msg_pub.positions.data);
    arm_msg_pub.positions.data = NULL;

    // hand publisher
    RCSOFTCHECK(rcl_publisher_fini(&hand_pub, &node));
    RCSOFTCHECK(rcl_timer_fini(&hand_timer));
    RCSOFTCHECK(rclc_executor_fini(&hand_executor_pub));

    free(hand_msg_pub.positions.data);
    hand_msg_pub.positions.data = NULL;

    // common
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

void microROSTaskFunction(void *parameter) {
    // Use inifiite loop to keep the task running like the void loop() function in Arduino framework.
    while (true) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                };
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&arm_executor_sub, RCL_MS_TO_NS(100));
                    rclc_executor_spin_some(&hand_executor_sub, RCL_MS_TO_NS(100));
                    rclc_executor_spin_some(&arm_executor_pub, RCL_MS_TO_NS(100));
                    rclc_executor_spin_some(&hand_executor_pub, RCL_MS_TO_NS(100));
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
}

void armControlTaskFunction(void *parameter) {
    ArmManager armManager(uint8_t(NUM_ALL_SERVOS), servoMinAngles, servoMaxAngles, servoInitAngles);

    while (true) {
        for (size_t i = 0; i < NUM_ALL_SERVOS; ++i) {
            armManager.setServoTargetAngle(i, uint8_t(joint_positions[i]));
        }
        armManager.moveArm();

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(100);

    state = WAITING_AGENT;

    // Initialize joint_positions with the initial angles
    for (size_t i = 0; i < NUM_ALL_SERVOS; ++i) {
        joint_positions[i] = double(servoInitAngles[i]);
    }

    xTaskCreate(
        microROSTaskFunction,      // Task function
        "Micro ROS Task",          // Task name
        8192,                      // Stack size (in bytes)
        NULL,                      // Task parameters
        configMAX_PRIORITIES - 1,  // Task priority
        NULL                       // Task handle
    );
    delay(100);
    xTaskCreate(
        armControlTaskFunction,  // Task function
        "Arm Control Task",      // Task name
        4096,                    // Stack size (in bytes)
        NULL,                    // Task parameters
        2,                       // Task priority
        NULL                     // Task handle
    );
    delay(100);
    xTaskCreate(
        led_task,    // Task function
        "LED Task",  // Task name
        1024,        // Stack size (in bytes)
        &state,      // Task parameters
        0,           // Task priority
        NULL         // Task handle
    );
    delay(100);
}

void loop() {
    // We use xTaskCreate and thus we don't need to put anything here.
}
