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
rcl_subscription_t sub;
trajectory_msgs__msg__JointTrajectoryPoint msg_sub;
rclc_executor_t executor_sub;

// publisher
rcl_publisher_t pub;
trajectory_msgs__msg__JointTrajectoryPoint msg_pub;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// Global variables shared between the microROS task and the arm control task
double joint_positions[NUM_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

states state;

void subscription_callback(const void *msgin) {
    const trajectory_msgs__msg__JointTrajectoryPoint *msg = (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;
    for (size_t i = 0; i < msg->positions.size; ++i) {
        joint_positions[i] = degrees(msg->positions.data[i]);
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&pub, &msg_pub, NULL));
        for (size_t i = 0; i < msg_pub.positions.capacity; i++) {
            msg_pub.positions.data[i] = joint_positions[i];
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

    // create subscriber
    FNCHECK(create_subscriber(&allocator, &support, &node, TOPIC_NAME, &sub, subscription_callback, &msg_sub, NUM_SERVOS, &executor_sub), bool);

    // create publisher
    FNCHECK(create_publisher(&allocator, &support, &node, REPUBLISH_TOPIC_NAME, &pub, &timer, timer_callback, &msg_pub, NUM_SERVOS, &executor_pub), bool);

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // subscriber
    delete_subscriber(&node, &sub, &msg_sub, &executor_sub);

    // publisher
    delete_publisher(&node, &pub, &timer, &msg_pub, &executor_pub);

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
                }
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(20, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(50));
                    rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(50));
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
    ArmManager armManager(uint8_t(NUM_SERVOS), servoMinAngles, servoMaxAngles, servoInitAngles);

    while (true) {
        for (size_t i = 0; i < NUM_SERVOS; ++i) {
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

    state = WAITING_AGENT;

    // Initialize joint_positions with the initial angles
    for (size_t i = 0; i < NUM_SERVOS; ++i) {
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
