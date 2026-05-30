/**
 * @file sensor_task.c
 * @brief Reads DHT20, TC74 and LDR, updates shared sensor data struct.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "shared_data.h"
#include "tasks/sensor_task.h"

static const char *TAG = "SENSOR";

/* ── TC74 registers ──────────────────────────────────────────────────── */
#define TC74_REG_TEMP    0x00
#define TC74_REG_CONFIG  0x01
#define TC74_TIMEOUT_MS  1000

/* ── LDR ADC channel ─────────────────────────────────────────────────── */
#define LDR_ADC_CHANNEL  ADC_CHANNEL_1

/* ═══════════════════════════════════════════════════════════════════════
 * TC74
 * ═══════════════════════════════════════════════════════════════════════ */
static void tc74_wake(void)
{
    uint8_t reg = TC74_REG_CONFIG, cfg = 0;
    if (i2c_master_transmit_receive(g_tc74_dev, &reg, 1, &cfg, 1,
                                    TC74_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) {
        ESP_LOGW(TAG, "TC74: cannot read config");
        return;
    }
    if (cfg & 0x80) {
        uint8_t buf[2] = { TC74_REG_CONFIG, cfg & 0x7F };
        i2c_master_transmit(g_tc74_dev, buf, 2,
                            TC74_TIMEOUT_MS / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "TC74: woke from standby");
    } else {
        ESP_LOGI(TAG, "TC74: ready (config=0x%02X)", cfg);
    }
}

static esp_err_t tc74_read(int8_t *out)
{
    uint8_t reg = TC74_REG_TEMP, raw = 0;
    esp_err_t err = i2c_master_transmit_receive(g_tc74_dev, &reg, 1, &raw, 1,
                                                TC74_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err == ESP_OK) *out = (int8_t)raw;
    return err;
}

/* ═══════════════════════════════════════════════════════════════════════
 * DHT20
 * ═══════════════════════════════════════════════════════════════════════ */
static esp_err_t dht20_read(float *out_temp, float *out_hum)
{
    uint8_t trigger[3] = { 0xAC, 0x33, 0x00 };
    esp_err_t err = i2c_master_transmit(g_dht20_dev, trigger, 3,
                                        TC74_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DHT20: trigger failed (0x%x)", err);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(90));

    uint8_t data[7] = {0};
    err = i2c_master_receive(g_dht20_dev, data, 7,
                             TC74_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DHT20: read failed (0x%x)", err);
        return err;
    }
    if (data[0] & 0x80) return ESP_ERR_TIMEOUT;

    uint32_t raw_hum  = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *out_hum  = ((float)raw_hum  / 1048576.0f) * 100.0f;
    *out_temp = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
 * LDR
 * ═══════════════════════════════════════════════════════════════════════ */
static void ldr_read(int *raw, int *mv)
{
    adc_oneshot_read(g_adc_handle, LDR_ADC_CHANNEL, raw);
    if (g_adc_cali_ok)
        adc_cali_raw_to_voltage(g_adc_cali, *raw, mv);
    else
        *mv = (int)((*raw / 4095.0f) * 3300.0f);
}

/* ═══════════════════════════════════════════════════════════════════════
 * State derivation
 * ═══════════════════════════════════════════════════════════════════════ */
static room_state_t derive_state(float temp)
{
    if (temp < TEMP_COLD_THRESHOLD)  return STATE_COLD;
    if (temp > TEMP_ALERT_THRESHOLD) return STATE_ALERT;
    return STATE_NORMAL;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Task
 * ═══════════════════════════════════════════════════════════════════════ */
void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");
    tc74_wake();

    while (1) {
        float dht_temp = 0, dht_hum = 0;
        int8_t tc74_temp = 0;
        int ldr_raw = 0, ldr_mv = 0;

        bool dht_ok  = (dht20_read(&dht_temp, &dht_hum) == ESP_OK);
        bool tc74_ok = (tc74_read(&tc74_temp) == ESP_OK);
        ldr_read(&ldr_raw, &ldr_mv);

        /* Use DHT20 temp for state, fall back to TC74 if DHT20 fails */
        float ref_temp = dht_ok ? dht_temp : (float)tc74_temp;

        /* Update shared data */
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        g_sensor_data.dht20_temp  = dht_temp;
        g_sensor_data.dht20_hum   = dht_hum;
        g_sensor_data.dht20_ok    = dht_ok;
        g_sensor_data.tc74_temp   = tc74_temp;
        g_sensor_data.tc74_ok     = tc74_ok;
        g_sensor_data.ldr_raw     = ldr_raw;
        g_sensor_data.ldr_mv      = ldr_mv;
        g_sensor_data.state       = derive_state(ref_temp);
        g_sensor_data.last_update = (uint32_t)(esp_timer_get_time() / 1000);
        xSemaphoreGive(g_data_mutex);

        ESP_LOGI(TAG, "DHT20: %.2f°C %.1f%%  TC74: %d°C  LDR: %d  State: %d",
                 dht_temp, dht_hum, tc74_temp, ldr_raw, g_sensor_data.state);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
    }
}