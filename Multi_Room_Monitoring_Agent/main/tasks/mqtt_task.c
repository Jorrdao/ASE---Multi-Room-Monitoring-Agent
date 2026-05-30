/**
 * @file mqtt_task.c
 * @brief WiFi connect → publish → WiFi disconnect cycle with sys_state updates.
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

#ifndef WIFI_SSID
#define WIFI_SSID      CONFIG_WIFI_SSID
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD  CONFIG_WIFI_PASSWORD
#endif

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com:1883"

#if ROOM_ID == 1
#define MQTT_TOPIC "ase/room1/sensors"
#else
#define MQTT_TOPIC "ase/room2/sensors"
#endif

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    5

static EventGroupHandle_t  s_wifi_eg;
static int                 s_retry = 0;
static bool                s_wifi_initialised = false;

static void set_sys_state(sys_state_t state)
{
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    g_sensor_data.sys_state = state;
    xSemaphoreGive(g_data_mutex);
}

static void wifi_handler(void *arg, esp_event_base_t base,
                         int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
            s_retry++;
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_retry = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_once(void)
{
    if (s_wifi_initialised) return;

    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_handler, NULL, NULL);

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));

    s_wifi_initialised = true;
    ESP_LOGI(TAG, "WiFi driver initialised");
}

static bool wifi_connect(void)
{
    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_retry = 0;
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(10000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
        return true;
    }
    ESP_LOGW(TAG, "WiFi connect failed");
    esp_wifi_stop();
    return false;
}

static void wifi_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    ESP_LOGI(TAG, "WiFi disconnected");
}

static bool mqtt_connected = false;

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (id == MQTT_EVENT_CONNECTED)    mqtt_connected = true;
    if (id == MQTT_EVENT_DISCONNECTED) mqtt_connected = false;
}

static void build_payload(sensor_data_t *d, char *buf, size_t len)
{
    const char *state_str;
    switch (d->state) {
        case STATE_COLD:   state_str = "COLD";   break;
        case STATE_ALERT:  state_str = "ALERT";  break;
        default:           state_str = "NORMAL"; break;
    }
    snprintf(buf, len,
             "{\"room\":%d,\"temp_dht\":%.2f,\"hum\":%.2f,"
             "\"temp_tc74\":%d,\"ldr\":%d,\"state\":\"%s\"}",
             ROOM_ID, d->dht20_temp, d->dht20_hum,
             d->tc74_temp, d->ldr_raw, state_str);
}

static void mqtt_publish_and_disconnect(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    int wait = 0;
    while (!mqtt_connected && wait < 60) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait++;
    }

    if (mqtt_connected) {
        sensor_data_t local;
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(&local, &g_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(g_data_mutex);

        char payload[256];
        build_payload(&local, payload, sizeof(payload));

        int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC,
                                              payload, 0, 1, 0);
        if (msg_id >= 0)
            ESP_LOGI(TAG, "Published: %s", payload);
        else
            ESP_LOGW(TAG, "Publish failed");

        vTaskDelay(pdMS_TO_TICKS(500));
    } else {
        ESP_LOGW(TAG, "MQTT broker unreachable");
    }

    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    mqtt_connected = false;
}

void mqtt_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MQTT task started");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    wifi_init_once();

    while (1) {
        /* Connecting */
        set_sys_state(SYS_CONNECTING);

        if (wifi_connect()) {
            /* Publishing */
            set_sys_state(SYS_PUBLISHING);
            mqtt_publish_and_disconnect();
            wifi_disconnect();
        }

        /* Back to sleeping */
        set_sys_state(SYS_SLEEPING);

        vTaskDelay(pdMS_TO_TICKS(MQTT_INTERVAL_MS));
    }
}