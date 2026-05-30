/**
 * @file display_task.h
 * @brief Display task — renders sensor data on the TFT every second.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point.
 *        Updates TFT display every DISPLAY_INTERVAL_MS.
 */
void display_task(void *pvParameters);