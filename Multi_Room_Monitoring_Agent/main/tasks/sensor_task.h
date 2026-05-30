/**
 * @file sensor_task.h
 * @brief Sensor task — reads DHT20, TC74 and LDR periodically.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

/* Peripheral handles defined in main.c */
extern i2c_master_dev_handle_t  g_tc74_dev;
extern i2c_master_dev_handle_t  g_dht20_dev;
extern adc_oneshot_unit_handle_t g_adc_handle;
extern adc_cali_handle_t         g_adc_cali;
extern bool                      g_adc_cali_ok;

/**
 * @brief FreeRTOS task entry point.
 *        Reads all sensors every SENSOR_INTERVAL_MS and updates g_sensor_data.
 */
void sensor_task(void *pvParameters);