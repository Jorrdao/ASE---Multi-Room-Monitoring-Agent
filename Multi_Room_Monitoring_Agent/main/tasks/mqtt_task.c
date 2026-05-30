/**
 * @file mqtt_task.c
 * @brief WiFi connection + MQTT publish to HiveMQ public broker.
 *
 * Publishes JSON payload to:
 *   ase/room1/sensors  (or room2 depending on ROOM_ID)
 *
 * Payload format:
 *   {
 *     "room": 1,
 *     "temp_dht": 27.3,
 *     "hum": 56.2,
 *     "temp_tc74": 27,
 *     "ldr": 1024,
 *     "state": "NORMAL"
 *   }
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "shared_data.h"
#include "tasks/mqtt_task.h"

static const char *TAG = "MQTT";

/* ── WiFi credentials — set via menuconfig ───────────────────────────── */
#ifndef WIFI_SSID
#define WIFI_SSID      CONFIG_WIFI_SSID
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD  CONFIG_WIFI_PASSWORD
#endif

/* ── HiveMQ public broker ────────────────────────────────────────────── */
#define MQTT_BROKER_URI "mqtt://broker.hivemq.com:1883"

/* ── MQTT topic ──────────────────────────────────────────────────────── */
#if ROOM_ID == 1
#define MQTT_TOPIC      "ase/room1/sensors"
#else
#define MQTT_TOPIC      "ase/room2/sensors"
#endif

/* ── WiFi event group ────────────────────────────────────────────────── */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    5

static EventGroupHandle_t s_wifi_event_group;
static int                s_retry_count = 0;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;

/* ═══════════════════════════════════════════════════════════════════════
 * WiFi event handler
 * ═══════════════════════════════════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_mqtt_connected = false;
        if (s_retry_count < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "WiFi disconnected, retrying (%d/%d)",
                     s_retry_count, WIFI_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRIES);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * WiFi init
 * ═══════════════════════════════════════════════════════════════════════ */
static bool wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Wait for connection */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        return true;
    }
    ESP_LOGE(TAG, "WiFi init failed");
    return false;
}

/* ═══════════════════════════════════════════════════════════════════════
 * MQTT event handler
 * ═══════════════════════════════════════════════════════════════════════ */
static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected to %s", MQTT_BROKER_URI);
            s_mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT published msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MQTT init
 * ═══════════════════════════════════════════════════════════════════════ */
static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Build JSON payload
 * ═══════════════════════════════════════════════════════════════════════ */
static void build_payload(sensor_data_t *d, char *buf, size_t len)
{
    const char *state_str;
    switch (d->state) {
        case STATE_COLD:   state_str = "COLD";   break;
        case STATE_ALERT:  state_str = "ALERT";  break;
        default:           state_str = "NORMAL"; break;
    }

    snprintf(buf, len,
             "{"
             "\"room\":%d,"
             "\"temp_dht\":%.2f,"
             "\"hum\":%.2f,"
             "\"temp_tc74\":%d,"
             "\"ldr\":%d,"
             "\"state\":\"%s\""
             "}",
             ROOM_ID,
             d->dht20_temp,
             d->dht20_hum,
             d->tc74_temp,
             d->ldr_raw,
             state_str);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Task
 * ═══════════════════════════════════════════════════════════════════════ */
void mqtt_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MQTT task started");

    /* NVS required for WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    if (!wifi_init()) {
        ESP_LOGE(TAG, "WiFi failed — MQTT task exiting");
        vTaskDelete(NULL);
        return;
    }

    mqtt_init();

    /* Wait for MQTT connection */
    int wait = 0;
    while (!s_mqtt_connected && wait < 30) {
        vTaskDelay(pdMS_TO_TICKS(500));
        wait++;
    }

    if (!s_mqtt_connected) {
        ESP_LOGE(TAG, "MQTT broker unreachable — task exiting");
        vTaskDelete(NULL);
        return;
    }

    sensor_data_t local_data = {0};
    char payload[256];

    while (1) {
        /* Copy shared data */
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(&local_data, &g_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(g_data_mutex);

        if (s_mqtt_connected) {
            build_payload(&local_data, payload, sizeof(payload));
            int msg_id = esp_mqtt_client_publish(s_mqtt_client,
                                                  MQTT_TOPIC,
                                                  payload, 0,
                                                  1,    /* QoS 1 */
                                                  0);   /* not retained */
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Published to %s: %s", MQTT_TOPIC, payload);
            } else {
                ESP_LOGW(TAG, "Publish failed");
            }
        } else {
            ESP_LOGW(TAG, "MQTT not connected, skipping publish");
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_INTERVAL_MS));
    }
}