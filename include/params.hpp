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
///< End define

#ifndef IS_RIGHT
#define ROS_DOMAIN_ID 0
#define NODE_NAME "micro_ros_platformio_left_node"
#define NAMESPACE ""
#define TOPIC_NAME "/left"
#define REPUBLISH_TOPIC_NAME "/left_republish"
const uint8_t servoMinAngles[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 90, 180, 120, 180, 180, 180, 180, 180, 180, 180};
const uint8_t servoInitAngles[] = {170, 10, 100, 10, 100, 100, 180, 0, 0, 0, 0};
#else
#define ROS_DOMAIN_ID 0
#define NODE_NAME "micro_ros_platformio_right_node"
#define NAMESPACE ""
#define TOPIC_NAME "/right"
#define REPUBLISH_TOPIC_NAME "/right_republish"
const uint8_t servoMinAngles[] = {0, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 180, 180, 120, 180, 180, 180, 180, 180, 180, 180};
const uint8_t servoInitAngles[] = {10, 170, 80, 10, 80, 80, 0, 180, 180, 180, 180};
#endif

#define ESP32_LED 2
#define UPDATE_ARM_DELAY 1.0
const float ARM_MOVEMENT_STEP = 1.0;
const size_t NUM_ALL_SERVOS = 11;

static_assert(sizeof(servoMinAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(sizeof(servoMaxAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(sizeof(servoInitAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
