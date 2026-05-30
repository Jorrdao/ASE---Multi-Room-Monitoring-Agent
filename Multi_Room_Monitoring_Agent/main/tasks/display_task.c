/**
 * @file display_task.c
 * @brief Renders sensor data and room state on the ST7735 TFT display.
 *
 * Layout (160x80 landscape):
 *   Line 0 (y=2):  "ROOM X          HH:MM" (room id + uptime)
 *   Divider line   (y=12)
 *   Line 1 (y=16): "TEMP  XX.X C  / XX C"  (DHT20 / TC74)
 *   Line 2 (y=28): "HUM   XX.X %"
 *   Line 3 (y=40): "LDR   XXXX"
 *   Divider line   (y=52)
 *   Line 4 (y=56): "STATUS: NORMAL"         (coloured by state)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "st7735.h"
#include "shared_data.h"
#include "tasks/display_task.h"

static const char *TAG = "DISPLAY";

/* ── Colours ─────────────────────────────────────────────────────────── */
#define COL_BG      ST7735_BLACK
#define COL_TITLE   ST7735_CYAN
#define COL_LABEL   ST7735_WHITE
#define COL_VALUE   ST7735_YELLOW
#define COL_COLD    ST7735_BLUE
#define COL_NORMAL  ST7735_GREEN
#define COL_ALERT   ST7735_RED
#define COL_DIV     ST7735_GRAY

/* ── Layout ──────────────────────────────────────────────────────────── */
#define X_LABEL     2
#define X_VALUE     60
#define Y_HEADER    2
#define Y_DIV1      13
#define Y_TEMP      17
#define Y_HUM       29
#define Y_LDR       41
#define Y_DIV2      53
#define Y_STATUS    57

/* ── Helpers ─────────────────────────────────────────────────────────── */
static void draw_divider(uint16_t y, uint16_t color)
{
    for (uint16_t x = 0; x < ST7735_WIDTH; x++) {
        st7735_draw_pixel(x, y, color);
    }
}

static void clear_row(uint16_t y, uint16_t h)
{
    st7735_fill_rect(0, y, ST7735_WIDTH, h, COL_BG);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Full screen draw (called once on startup)
 * ═══════════════════════════════════════════════════════════════════════ */
static void draw_static_layout(void)
{
    st7735_fill_screen(COL_BG);

    /* Header */
    char header[24];
    snprintf(header, sizeof(header), "ROOM %d", ROOM_ID);
    st7735_draw_string(X_LABEL, Y_HEADER, header, COL_TITLE, COL_BG, 1);

    /* Dividers */
    draw_divider(Y_DIV1, COL_DIV);
    draw_divider(Y_DIV2, COL_DIV);

    /* Labels */
    st7735_draw_string(X_LABEL, Y_TEMP,   "TEMP",   COL_LABEL, COL_BG, 1);
    st7735_draw_string(X_LABEL, Y_HUM,    "HUM",    COL_LABEL, COL_BG, 1);
    st7735_draw_string(X_LABEL, Y_LDR,    "LDR",    COL_LABEL, COL_BG, 1);
    st7735_draw_string(X_LABEL, Y_STATUS, "STAT",   COL_LABEL, COL_BG, 1);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Dynamic value updates
 * ═══════════════════════════════════════════════════════════════════════ */
static void update_values(sensor_data_t *d)
{
    char buf[32];

    /* Uptime in seconds top right */
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    snprintf(buf, sizeof(buf), "%lus", (unsigned long)uptime_s);
    clear_row(Y_HEADER, 10);
    char header[24];
    snprintf(header, sizeof(header), "ROOM %d", ROOM_ID);
    st7735_draw_string(X_LABEL, Y_HEADER, header,  COL_TITLE, COL_BG, 1);
    st7735_draw_string(110,     Y_HEADER, buf,      COL_DIV,   COL_BG, 1);

    /* Temperature */
    clear_row(Y_TEMP, 11);
    st7735_draw_string(X_LABEL, Y_TEMP, "TEMP", COL_LABEL, COL_BG, 1);
    if (d->dht20_ok) {
        snprintf(buf, sizeof(buf), "%.1fC", d->dht20_temp);
        st7735_draw_string(X_VALUE, Y_TEMP, buf, COL_VALUE, COL_BG, 1);
    } else {
        st7735_draw_string(X_VALUE, Y_TEMP, "---", COL_DIV, COL_BG, 1);
    }
    /* TC74 secondary temp */
    if (d->tc74_ok) {
        snprintf(buf, sizeof(buf), "/%dC", d->tc74_temp);
        st7735_draw_string(110, Y_TEMP, buf, COL_DIV, COL_BG, 1);
    }

    /* Humidity */
    clear_row(Y_HUM, 11);
    st7735_draw_string(X_LABEL, Y_HUM, "HUM", COL_LABEL, COL_BG, 1);
    if (d->dht20_ok) {
        snprintf(buf, sizeof(buf), "%.1f%%", d->dht20_hum);
        st7735_draw_string(X_VALUE, Y_HUM, buf, COL_VALUE, COL_BG, 1);
    } else {
        st7735_draw_string(X_VALUE, Y_HUM, "---", COL_DIV, COL_BG, 1);
    }

    /* LDR */
    clear_row(Y_LDR, 11);
    st7735_draw_string(X_LABEL, Y_LDR, "LDR", COL_LABEL, COL_BG, 1);
    snprintf(buf, sizeof(buf), "%d", d->ldr_raw);
    st7735_draw_string(X_VALUE, Y_LDR, buf, COL_VALUE, COL_BG, 1);

    /* Status */
    clear_row(Y_STATUS, 11);
    st7735_draw_string(X_LABEL, Y_STATUS, "STAT", COL_LABEL, COL_BG, 1);
    switch (d->state) {
        case STATE_COLD:
            st7735_draw_string(X_VALUE, Y_STATUS, "COLD",   COL_COLD,   COL_BG, 1);
            break;
        case STATE_NORMAL:
            st7735_draw_string(X_VALUE, Y_STATUS, "NORMAL", COL_NORMAL, COL_BG, 1);
            break;
        case STATE_ALERT:
            st7735_draw_string(X_VALUE, Y_STATUS, "ALERT!", COL_ALERT,  COL_BG, 1);
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Task
 * ═══════════════════════════════════════════════════════════════════════ */
void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");

    draw_static_layout();

    sensor_data_t local_data = {0};

    while (1) {
        /* Copy shared data */
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(&local_data, &g_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(g_data_mutex);

        update_values(&local_data);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
}