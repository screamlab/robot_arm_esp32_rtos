#pragma once
#include <stdint.h>

static constexpr uint8_t NUM_OF_SERVOS = 6;
static constexpr uint8_t servoMinAngles[] = {0, 80, 0, 0, 0, 0};
static constexpr uint8_t servoMaxAngles[] = {180, 180, 180, 120, 180, 90};
static constexpr uint8_t servoInitAngles[] = {10, 170, 80, 60, 80, 10};