#pragma once
#include <stdint.h>

/*
 * https://circuits4you.com
 * ESP32 LED Blink Example
 * Board ESP23 DEVKIT V1
 *
 * ON Board LED GPIO 2
 * Ref: https://circuits4you.com/2018/02/02/esp32-led-blink-example/
 */

///< Begin define
// Define the left or right arm
#define IS_RIGHT  // Comment this line to use left hand

// Define whether to use mutex lock or not
#define USE_MUTEX_LOCK  // Comment this line to disable mutex lock

// Define whether to publish the data in the subscription callback
// #define USE_PUBLISH  // Comment this line to disable publishing in the subscription callback
///< End define

#ifndef IS_RIGHT
#define ROS_DOMAIN_ID 0
#define NODE_NAME "micro_ros_platformio_left_node"
#define NAMESPACE ""
#define ARM_TOPIC_NAME "/left_arm"
#define HAND_TOPIC_NAME "/left_hand"
#define ARM_REPUBLISH_TOPIC_NAME "/left_arm_republish"
#define HAND_REPUBLISH_TOPIC_NAME "/left_hand_republish"
const uint8_t servoMinAngles[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 90, 180, 120, 180, 180, 180, 180, 180, 180, 180};
const uint8_t servoInitAngles[] = {170, 10, 100, 10, 100, 100, 180, 0, 0, 0, 0};
#else
#define ROS_DOMAIN_ID 0
#define NODE_NAME "micro_ros_platformio_right_node"
#define NAMESPACE ""
#define ARM_TOPIC_NAME "/right_arm"
#define HAND_TOPIC_NAME "/right_hand"
#define ARM_REPUBLISH_TOPIC_NAME "/right_arm_republish"
#define HAND_REPUBLISH_TOPIC_NAME "/right_hand_republish"
const uint8_t servoMinAngles[] = {0, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 180, 180, 120, 180, 180, 180, 180, 180, 180, 180};
const uint8_t servoInitAngles[] = {10, 170, 80, 10, 80, 80, 0, 180, 180, 180, 180};
#endif

#define ESP32_LED 2
#define UPDATE_ARM_DELAY 1.0
const float ARM_MOVEMENT_STEP = 1.0;
const size_t NUM_ALL_SERVOS = 11;
const size_t NUM_ARM_SERVOS = 6;
const size_t NUM_HAND_SERVOS = 5;
const size_t ARM_OFFSET = 0;
const size_t HAND_OFFSET = NUM_ARM_SERVOS;

static_assert(sizeof(servoMinAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(sizeof(servoMaxAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(sizeof(servoInitAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(NUM_ARM_SERVOS + NUM_HAND_SERVOS == NUM_ALL_SERVOS);

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
