#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

#include <armDriver.hpp>
#include <esp32_led.hpp>
#include <params.hpp>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

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

rcl_allocator_t allocator;
rcl_init_options_t init_options;
rclc_support_t support;
rcl_node_t node;

// subscriber
rcl_subscription_t sub;
trajectory_msgs__msg__JointTrajectoryPoint msg_sub;

// publisher
rcl_publisher_t pub;
trajectory_msgs__msg__JointTrajectoryPoint msg_pub;
rcl_timer_t timer;
uint64_t timer_timeout = RCL_MS_TO_NS(50);  // Timer period in milliseconds

// executor
rclc_executor_t executor;

// Global mutex declaration
#ifdef USE_MUTEX_LOCK
SemaphoreHandle_t xJointPositionsMutex = NULL;
#endif

// Global variables shared between the microROS task and the arm control task
double joint_positions[NUM_ALL_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
uint64_t msg_cnt = 0;

states state;

void subscription_callback(const void *msgin) {
    const trajectory_msgs__msg__JointTrajectoryPoint *msg = (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;
#ifdef USE_MUTEX_LOCK
    // Attempt to lock the mutex
    if (xSemaphoreTake(xJointPositionsMutex, portMAX_DELAY) == pdTRUE) {
#endif
        for (size_t i = 0; i < msg->positions.size; ++i) {
            joint_positions[i] = degrees(msg->positions.data[i]);
        }
        Serial2.printf("#%llu: ", ++msg_cnt);
        for (size_t i = 0; i < msg->positions.size; ++i) {
            Serial2.printf("%d%c", (int)msg->positions.data[i], (i == msg->positions.size - 1) ? '\n' : ' ');
        }
#ifdef USE_MUTEX_LOCK
        // Release the mutex after update
        xSemaphoreGive(xJointPositionsMutex);
    }
#endif
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
#ifdef USE_MUTEX_LOCK
        // Attempt to lock the mutex
        if (xSemaphoreTake(xJointPositionsMutex, portMAX_DELAY) == pdTRUE) {
#endif
            for (size_t i = 0; i < msg_pub.positions.capacity; i++) {
                msg_pub.positions.data[i] = radians(joint_positions[i]);
            }
#ifdef USE_MUTEX_LOCK
            // Release the mutex after update
            xSemaphoreGive(xJointPositionsMutex);
        }
#endif
        RCSOFTCHECK(rcl_publish(&pub, &msg_pub, NULL));
    }
}

/**
 * Functions create_entities and destroy_entities can take several seconds.
 * In order to reduce this rebuild the library with
 * - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
 * - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
 */

bool create_entities() {
    // Use the default allocator
    allocator = rcl_get_default_allocator();

    // Init the init options with the default allocator and set the domain id
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // Init the node support with options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                           &allocator));

    // Init the node with the node support
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, NAMESPACE, &support));

    // Allocate the memory for the messages
    msg_sub.positions.capacity = NUM_ALL_SERVOS;
    msg_sub.positions.size = NUM_ALL_SERVOS;
    msg_sub.positions.data = (double *)calloc(msg_sub.positions.capacity, sizeof(double));

    msg_pub.positions.capacity = NUM_ALL_SERVOS;
    msg_pub.positions.size = NUM_ALL_SERVOS;
    msg_pub.positions.data = (double *)calloc(msg_pub.positions.capacity, sizeof(double));

    // Init the subscriber on the best effort mode
    RCCHECK(rclc_subscription_init_default(
        &sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        TOPIC_NAME));

    // Init the publisher on the best effort mode
    RCCHECK(rclc_publisher_init_default(
        &pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        REPUBLISH_TOPIC_NAME));

    // Init the executor with the node support
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    // Add the subscription to the executor
    RCCHECK(rclc_executor_add_subscription(&executor, &sub, &msg_sub,
                                           subscription_callback, ON_NEW_DATA));

    // Init timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        timer_timeout,  // Timer period in nanoseconds
        timer_callback));

    // Add the timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_subscription_fini(&sub, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    trajectory_msgs__msg__JointTrajectoryPoint__fini(&msg_sub);
    trajectory_msgs__msg__JointTrajectoryPoint__fini(&msg_pub);
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    RCSOFTCHECK(rcl_init_options_fini(&init_options));
}

void microROSTaskFunction(void *parameter) {
    static uint64_t cnt = 0;
    // Use inifiite loop to keep the task running like the void loop() function in Arduino framework.
    while (true) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(20, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                Serial2.print("Waiting for agent");
                for (uint64_t i = 0; i < cnt % 3; ++i) {
                    Serial2.print(' ');
                }
                Serial2.print("...\n");
                ++cnt;
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
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
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

#ifdef USE_MUTEX_LOCK
    // Create the mutex
    do {
        xJointPositionsMutex = xSemaphoreCreateMutex();
    } while (xJointPositionsMutex == NULL);
#endif

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
