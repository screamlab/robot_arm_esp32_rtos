#include <Arduino.h>

#include <armDriver.hpp>
#include <esp32_led.hpp>
#include <micro_ros_utils.hpp>
#include <params.hpp>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_allocator_t allocator;
rcl_init_options_t init_options;
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
uint64_t msg_cnt = 0;

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
    Serial2.printf("#%llu:\n", ++msg_cnt);
    for (size_t i = 0; i < msg->positions.size; ++i) {
        Serial2.printf("%.3lf%c", msg->positions.data[i], (i == msg->positions.size - 1 || i % 5 == 4) ? '\n' : ' ');
    }
    Serial2.println();
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
    // create node
    FNCHECK(create_node(&allocator, &init_options, &support, &node), bool);

    // create arm subscriber
    FNCHECK(create_subscriber(&allocator, &support, &node, ARM_TOPIC_NAME, &arm_sub, arm_subscription_callback, &arm_msg_sub, NUM_ARM_SERVOS, &arm_executor_sub), bool);

    // create hand subscriber
    FNCHECK(create_subscriber(&allocator, &support, &node, HAND_TOPIC_NAME, &hand_sub, hand_subscription_callback, &hand_msg_sub, NUM_HAND_SERVOS, &hand_executor_sub), bool);

    // create arm publisher
    FNCHECK(create_publisher(&allocator, &support, &node, ARM_REPUBLISH_TOPIC_NAME, &arm_pub, &arm_timer, arm_timer_callback, &arm_msg_pub, NUM_ARM_SERVOS, &arm_executor_pub), bool);

    // create hand publisher
    FNCHECK(create_publisher(&allocator, &support, &node, HAND_REPUBLISH_TOPIC_NAME, &hand_pub, &hand_timer, hand_timer_callback, &hand_msg_pub, NUM_HAND_SERVOS, &hand_executor_pub), bool);

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // arm subscriber
    delete_subscriber(&node, &arm_sub, &arm_msg_sub, &arm_executor_sub);

    // hand subscriber
    delete_subscriber(&node, &hand_sub, &hand_msg_sub, &hand_executor_sub);

    // arm publisher
    delete_publisher(&node, &arm_pub, &arm_timer, &arm_msg_pub, &arm_executor_pub);

    // hand publisher
    delete_publisher(&node, &hand_pub, &hand_timer, &hand_msg_pub, &hand_executor_pub);

    // common
    delete_node(&init_options, &support, &node);
}

void microROSTaskFunction(void *parameter) {
    // Use inifiite loop to keep the task running like the void loop() function in Arduino framework.
    while (true) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(20, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                };
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(20, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
    Serial.begin(921600);
    set_microros_serial_transports(Serial);
    delay(100);

    // Configure UART2 serial transport for debugging
    Serial2.begin(921600, SERIAL_8N1, RX2, TX2);

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
