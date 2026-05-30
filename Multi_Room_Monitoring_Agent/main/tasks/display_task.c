/**
 * @file display_task.c
 * @brief Renders sensor data and system state on the ST7735 TFT.
 *
 * Two screen modes:
 *  - DATA screen:    shown when SYS_AWAKE — sensor values + room state
 *  - STATUS screen:  shown for all other sys states — large room ID +
 *                    big state indicator (SLEEPING / CONNECTING / PUBLISHING)
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
#define COL_BG          ST7735_BLACK
#define COL_WHITE       ST7735_WHITE
#define COL_GRAY        ST7735_GRAY
#define COL_CYAN        ST7735_CYAN
#define COL_COLD        ST7735_BLUE
#define COL_NORMAL      ST7735_GREEN
#define COL_ALERT       ST7735_RED
#define COL_ORANGE      ST7735_RGB565(234, 88, 12)
#define COL_YELLOW      ST7735_YELLOW
#define COL_PURPLE      ST7735_RGB565(147, 51, 234)

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

/* ── Track last drawn state to avoid full redraws ────────────────────── */
static sys_state_t  s_last_sys  = (sys_state_t)-1;
static room_state_t s_last_room = (room_state_t)-1;

/* ═══════════════════════════════════════════════════════════════════════
 * Helpers
 * ═══════════════════════════════════════════════════════════════════════ */
static void draw_divider(uint16_t y, uint16_t color)
{
    for (uint16_t x = 0; x < ST7735_WIDTH; x++)
        st7735_draw_pixel(x, y, color);
}

static void clear_row(uint16_t y, uint16_t h)
{
    st7735_fill_rect(0, y, ST7735_WIDTH, h, COL_BG);
}

/* ═══════════════════════════════════════════════════════════════════════
 * DATA screen — shown when AWAKE
 * ═══════════════════════════════════════════════════════════════════════ */
static void draw_data_static(void)
{
    st7735_fill_screen(COL_BG);

    char header[24];
    snprintf(header, sizeof(header), "ROOM %d", ROOM_ID);
    st7735_draw_string(X_LABEL, Y_HEADER, header, COL_CYAN, COL_BG, 1);

    draw_divider(Y_DIV1, COL_GRAY);
    draw_divider(Y_DIV2, COL_GRAY);

    st7735_draw_string(X_LABEL, Y_TEMP,   "TEMP", COL_WHITE, COL_BG, 1);
    st7735_draw_string(X_LABEL, Y_HUM,    "HUM",  COL_WHITE, COL_BG, 1);
    st7735_draw_string(X_LABEL, Y_LDR,    "LDR",  COL_WHITE, COL_BG, 1);
    st7735_draw_string(X_LABEL, Y_STATUS, "STAT", COL_WHITE, COL_BG, 1);
}

static void update_data_values(sensor_data_t *d)
{
    char buf[32];

    /* Uptime */
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    snprintf(buf, sizeof(buf), "%lus", (unsigned long)uptime_s);
    clear_row(Y_HEADER, 10);
    char header[24];
    snprintf(header, sizeof(header), "ROOM %d", ROOM_ID);
    st7735_draw_string(X_LABEL, Y_HEADER, header, COL_CYAN,  COL_BG, 1);
    st7735_draw_string(110,     Y_HEADER, buf,    COL_GRAY,  COL_BG, 1);

    /* Temperature */
    clear_row(Y_TEMP, 11);
    st7735_draw_string(X_LABEL, Y_TEMP, "TEMP", COL_WHITE, COL_BG, 1);
    uint16_t temp_col = (d->state == STATE_COLD)  ? COL_COLD  :
                        (d->state == STATE_ALERT) ? COL_ALERT : COL_NORMAL;
    if (d->dht20_ok) {
        snprintf(buf, sizeof(buf), "%.1fC", d->dht20_temp);
        st7735_draw_string(X_VALUE, Y_TEMP, buf, temp_col, COL_BG, 1);
    } else {
        st7735_draw_string(X_VALUE, Y_TEMP, "---", COL_GRAY, COL_BG, 1);
    }
    if (d->tc74_ok) {
        snprintf(buf, sizeof(buf), "/%dC", d->tc74_temp);
        st7735_draw_string(110, Y_TEMP, buf, COL_GRAY, COL_BG, 1);
    }

    /* Humidity */
    clear_row(Y_HUM, 11);
    st7735_draw_string(X_LABEL, Y_HUM, "HUM", COL_WHITE, COL_BG, 1);
    if (d->dht20_ok) {
        snprintf(buf, sizeof(buf), "%.1f%%", d->dht20_hum);
        st7735_draw_string(X_VALUE, Y_HUM, buf, COL_YELLOW, COL_BG, 1);
    } else {
        st7735_draw_string(X_VALUE, Y_HUM, "---", COL_GRAY, COL_BG, 1);
    }

    /* LDR */
    clear_row(Y_LDR, 11);
    st7735_draw_string(X_LABEL, Y_LDR, "LDR", COL_WHITE, COL_BG, 1);
    snprintf(buf, sizeof(buf), "%d", d->ldr_raw);
    st7735_draw_string(X_VALUE, Y_LDR, buf, COL_YELLOW, COL_BG, 1);

    /* Status */
    clear_row(Y_STATUS, 11);
    st7735_draw_string(X_LABEL, Y_STATUS, "STAT", COL_WHITE, COL_BG, 1);
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
 * STATUS screen — shown for SLEEPING / CONNECTING / PUBLISHING
 * ═══════════════════════════════════════════════════════════════════════ */
static void draw_status_screen(sys_state_t sys, room_state_t room)
{
    st7735_fill_screen(COL_BG);

    /* Room ID — large, centred */
    char room_str[16];
    snprintf(room_str, sizeof(room_str), "ROOM %d", ROOM_ID);
    uint16_t rx = (ST7735_WIDTH - (strlen(room_str) * 12)) / 2;
    st7735_draw_string(rx, 8, room_str, COL_CYAN, COL_BG, 2);

    /* Divider */
    draw_divider(30, COL_GRAY);

    /* Big state label */
    const char *label    = "";
    uint16_t    label_col = COL_WHITE;

    switch (sys) {
        case SYS_SLEEPING:
            label     = "SLEEPING";
            label_col = COL_PURPLE;
            break;
        case SYS_CONNECTING:
            label     = "CONNECTING";
            label_col = COL_ORANGE;
            break;
        case SYS_PUBLISHING:
            label     = "PUBLISHING";
            label_col = COL_NORMAL;
            break;
        default:
            break;
    }

    /* Centre the label */
    uint16_t lx = (ST7735_WIDTH - (strlen(label) * 6)) / 2;
    st7735_draw_string(lx, 38, label, label_col, COL_BG, 1);

    /* Room env state bottom bar */
    draw_divider(54, COL_GRAY);

    const char *env_str;
    uint16_t    env_col;
    switch (room) {
        case STATE_COLD:   env_str = "COLD";   env_col = COL_COLD;   break;
        case STATE_ALERT:  env_str = "ALERT!"; env_col = COL_ALERT;  break;
        default:           env_str = "NORMAL"; env_col = COL_NORMAL; break;
    }
    uint16_t ex = (ST7735_WIDTH - (strlen(env_str) * 6)) / 2;
    st7735_draw_string(ex, 60, env_str, env_col, COL_BG, 1);

    /* Uptime bottom right */
    char up[16];
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    snprintf(up, sizeof(up), "%lus", (unsigned long)uptime_s);
    st7735_draw_string(120, 70, up, COL_GRAY, COL_BG, 1);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Task
 * ═══════════════════════════════════════════════════════════════════════ */
void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");

    /* Start with data screen layout */
    draw_data_static();

    sensor_data_t local = {0};
    bool data_screen_drawn = true;

    while (1) {
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(&local, &g_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(g_data_mutex);

        sys_state_t  sys  = local.sys_state;
        room_state_t room = local.state;

        if (sys == SYS_AWAKE) {
            /* Switch to data screen if we were on status screen */
            if (!data_screen_drawn || s_last_sys != SYS_AWAKE) {
                draw_data_static();
                data_screen_drawn = true;
            }
            update_data_values(&local);
        } else {
            /* Show status screen — redraw if state changed */
            if (sys != s_last_sys || room != s_last_room) {
                draw_status_screen(sys, room);
                data_screen_drawn = false;
            }
        }

        s_last_sys  = sys;
        s_last_room = room;

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
}