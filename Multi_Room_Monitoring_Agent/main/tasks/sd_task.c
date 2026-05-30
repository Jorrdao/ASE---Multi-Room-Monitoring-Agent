/**
 * @file sd_task.c
 * @brief Logs sensor data to CSV on the SD card.
 *
 * File: /sdcard/room1.csv  (or room2.csv)
 * Format:
 *   uptime_s,temp_dht,hum,temp_tc74,ldr,state
 *   123,27.30,56.20,27,1024,NORMAL
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "shared_data.h"
#include "tasks/sd_task.h"

static const char *TAG = "SD_TASK";

/* ── File path ───────────────────────────────────────────────────────── */
#if ROOM_ID == 1
#define SD_FILE  "/sdcard/room1.csv"
#else
#define SD_FILE  "/sdcard/room2.csv"
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * Write CSV header if file is new
 * ═══════════════════════════════════════════════════════════════════════ */
static void ensure_header(void)
{
    /* Check if file already exists */
    FILE *f = fopen(SD_FILE, "r");
    if (f != NULL) {
        fclose(f);
        return; /* already has header */
    }

    f = fopen(SD_FILE, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Cannot create %s", SD_FILE);
        return;
    }
    fprintf(f, "uptime_s,temp_dht,hum,temp_tc74,ldr,state\n");
    fclose(f);
    ESP_LOGI(TAG, "Created %s with header", SD_FILE);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Append one CSV row
 * ═══════════════════════════════════════════════════════════════════════ */
static void append_row(sensor_data_t *d)
{
    FILE *f = fopen(SD_FILE, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Cannot open %s for append", SD_FILE);
        return;
    }

    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);

    const char *state_str;
    switch (d->state) {
        case STATE_COLD:   state_str = "COLD";   break;
        case STATE_ALERT:  state_str = "ALERT";  break;
        default:           state_str = "NORMAL"; break;
    }

    fprintf(f, "%lu,%.2f,%.2f,%d,%d,%s\n",
            (unsigned long)uptime_s,
            d->dht20_temp,
            d->dht20_hum,
            d->tc74_temp,
            d->ldr_raw,
            state_str);

    fclose(f);
    ESP_LOGI(TAG, "Logged row to %s (uptime=%lus)", SD_FILE,
             (unsigned long)uptime_s);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Task
 * ═══════════════════════════════════════════════════════════════════════ */
void sd_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SD task started");

    /* Wait a bit for SD to be mounted by main.c */
    vTaskDelay(pdMS_TO_TICKS(2000));

    ensure_header();

    sensor_data_t local_data = {0};

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SD_INTERVAL_MS));

        /* Copy shared data */
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(&local_data, &g_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(g_data_mutex);

        append_row(&local_data);
    }
}