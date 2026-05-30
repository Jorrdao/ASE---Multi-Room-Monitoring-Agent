/**
 * @file mqtt_task.h
 * @brief MQTT task — connects to HiveMQ and publishes sensor data.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point.
 *        Connects to WiFi + HiveMQ broker and publishes every MQTT_INTERVAL_MS.
 */
void mqtt_task(void *pvParameters);