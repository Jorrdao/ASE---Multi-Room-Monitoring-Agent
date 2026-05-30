/**
 * @file shared_data.h
 * @brief Shared sensor data structure and synchronisation primitives.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ── Room identity ───────────────────────────────────────────────────── */
#ifndef ROOM_ID
#define ROOM_ID 1
#endif

/* ── Alert thresholds (°C) ───────────────────────────────────────────── */
#define TEMP_COLD_THRESHOLD     22.0f
#define TEMP_ALERT_THRESHOLD    30.0f

/* ── Intervals ───────────────────────────────────────────────────────── */
#define SENSOR_INTERVAL_MS      5000
#define DISPLAY_INTERVAL_MS     1000
#define MQTT_INTERVAL_MS        10000
#define SD_INTERVAL_MS          30000

/* ── Room environmental state ────────────────────────────────────────── */
typedef enum {
    STATE_COLD   = 0,
    STATE_NORMAL = 1,
    STATE_ALERT  = 2,
} room_state_t;

/* ── System operational state ────────────────────────────────────────── */
typedef enum {
    SYS_AWAKE      = 0,   /* reading sensors, normal operation */
    SYS_SLEEPING   = 1,   /* light sleep between reads         */
    SYS_CONNECTING = 2,   /* WiFi connecting                   */
    SYS_PUBLISHING = 3,   /* sending data to MQTT broker       */
} sys_state_t;

/* ── Shared sensor data ──────────────────────────────────────────────── */
typedef struct {
    /* DHT20 */
    float    dht20_temp;
    float    dht20_hum;
    bool     dht20_ok;

    /* TC74 */
    int8_t   tc74_temp;
    bool     tc74_ok;

    /* LDR */
    int      ldr_raw;
    int      ldr_mv;

    /* Derived */
    room_state_t state;
    sys_state_t  sys_state;
    uint32_t     last_update;
} sensor_data_t;

/* ── Globals (defined in main.c) ─────────────────────────────────────── */
extern sensor_data_t    g_sensor_data;
extern SemaphoreHandle_t g_data_mutex;