/**
 * @file sd_task.h
 * @brief SD task — logs sensor data to CSV on the SD card.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point.
 *        Appends a CSV row to /sdcard/roomX.csv every SD_INTERVAL_MS.
 */
void sd_task(void *pvParameters);