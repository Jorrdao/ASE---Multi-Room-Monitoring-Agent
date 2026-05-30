/**
 * @file shared_data.h
 * @brief Shared sensor data structure and synchronisation primitives.
 *
 * All FreeRTOS tasks read/write sensor data through this struct.
 * Access must be protected by g_data_mutex.
 *
 * Usage:
 *   xSemaphoreTake(g_data_mutex, portMAX_DELAY);
 *   // read or write g_sensor_data fields
 *   xSemaphoreGive(g_data_mutex);
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ── Room identity ───────────────────────────────────────────────────── */
#ifndef ROOM_ID
#define ROOM_ID 1   /* Change to 2 when flashing the second board */
#endif

/* ── Alert thresholds (°C) ───────────────────────────────────────────── */
#define TEMP_COLD_THRESHOLD     22.0f
#define TEMP_ALERT_THRESHOLD    30.0f

/* ── Sensor read interval ────────────────────────────────────────────── */
#define SENSOR_INTERVAL_MS      5000
#define DISPLAY_INTERVAL_MS     1000
#define MQTT_INTERVAL_MS        10000
#define SD_INTERVAL_MS          30000

/* ── Alert state ─────────────────────────────────────────────────────── */
typedef enum {
    STATE_COLD   = 0,
    STATE_NORMAL = 1,
    STATE_ALERT  = 2,
} room_state_t;

/* ── Shared sensor data ──────────────────────────────────────────────── */
typedef struct {
    /* DHT20 */
    float    dht20_temp;        /* °C */
    float    dht20_hum;         /* % */
    bool     dht20_ok;

    /* TC74 */
    int8_t   tc74_temp;         /* °C */
    bool     tc74_ok;

    /* LDR */
    int      ldr_raw;           /* 0-4095 */
    int      ldr_mv;            /* mV */

    /* Derived */
    room_state_t state;         /* COLD / NORMAL / ALERT */
    uint32_t     last_update;   /* ms since boot */
} sensor_data_t;

/* ── Globals (defined in main.c) ─────────────────────────────────────── */
extern sensor_data_t   g_sensor_data;
extern SemaphoreHandle_t g_data_mutex;