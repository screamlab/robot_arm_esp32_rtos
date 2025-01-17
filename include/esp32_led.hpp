#pragma once
#include <Arduino.h>

#include <params.hpp>

/**
 * @brief Task to control the LED based on the current state.
 *
 * This task will control the LED behavior based on the provided state:
 * - If the state is WAITING_AGENT, the LED will flash with a period of 0.25 ms.
 * - If the state is AGENT_CONNECTED, the LED will remain on.
 * - For any other state, the LED will remain off.
 *
 * @param parameter A pointer to an integer representing the current state.
 */
void led_task(void *parameter);
