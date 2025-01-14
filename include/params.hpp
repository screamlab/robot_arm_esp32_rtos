#pragma once
#include <stdint.h>

#define UPDATE_ARM_DELAY 1.0
const size_t NUM_OF_SERVOS = 6;
const uint8_t servoMinAngles[] = {0, 80, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 180, 180, 120, 180, 90};
const uint8_t servoInitAngles[] = {10, 170, 80, 60, 80, 10};
