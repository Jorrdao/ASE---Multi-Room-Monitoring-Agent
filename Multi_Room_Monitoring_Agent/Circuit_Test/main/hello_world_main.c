/*
 * circuit_test.c
 * Reads DHT20 (I2C), TC74 (I2C), and LDR (ADC) every 2 seconds.
 * Prints all values to serial monitor. No MQTT, no TFT, no SD.
 *
 * Pin assignments (from your schematic):
 *   I2C SDA  -> GPIO6
 *   I2C SCL  -> GPIO7
 *   LDR      -> GPIO2 (ADC1_CH2, through 10k divider to GND, top to 3.3V)
 *
 * TC74 I2C address: 0x48 (TC74A0)
 * DHT20 I2C address: 0x38
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

/* ── Pin definitions ─────────────────────────────────────────────────── */
#define I2C_SDA_PIN     GPIO_NUM_6
#define I2C_SCL_PIN     GPIO_NUM_7
#define I2C_FREQ_HZ     100000      /* 100 kHz standard mode */

#define LDR_ADC_CHANNEL ADC_CHANNEL_2   /* GPIO2 = ADC1_CH2 */
#define LDR_ADC_UNIT    ADC_UNIT_1
#define LDR_ADC_ATTEN   ADC_ATTEN_DB_12 /* 0-3.3 V range */

/* ── Sensor I2C addresses ────────────────────────────────────────────── */
#define TC74_ADDR       0x48
#define DHT20_ADDR      0x38

/* ── TC74 registers ──────────────────────────────────────────────────── */
#define TC74_REG_TEMP   0x00
#define TC74_REG_CONFIG 0x01

static const char *TAG = "CIRCUIT_TEST";

/* ── I2C handles ─────────────────────────────────────────────────────── */
static i2c_master_bus_handle_t  i2c_bus;
static i2c_master_dev_handle_t  tc74_dev;
static i2c_master_dev_handle_t  dht20_dev;

/* ── ADC handles ─────────────────────────────────────────────────────── */
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         adc_cali;
static bool                      adc_cali_ok = false;

/* ═══════════════════════════════════════════════════════════════════════
 * I2C init
 * ═══════════════════════════════════════════════════════════════════════ */
static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port       = I2C_NUM_0,
        .sda_io_num     = I2C_SDA_PIN,
        .scl_io_num     = I2C_SCL_PIN,
        .clk_source     = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* external 4.7k pull-ups on board */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* TC74 device */
    i2c_device_config_t tc74_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TC74_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &tc74_cfg, &tc74_dev));

    /* DHT20 device */
    i2c_device_config_t dht20_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DHT20_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dht20_cfg, &dht20_dev));

    ESP_LOGI(TAG, "I2C bus ready (SDA=GPIO%d, SCL=GPIO%d)", I2C_SDA_PIN, I2C_SCL_PIN);
}

/* ═══════════════════════════════════════════════════════════════════════
 * ADC init (LDR)
 * ═══════════════════════════════════════════════════════════════════════ */
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = LDR_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = LDR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &chan_cfg));

    /* Try to enable calibration (curve fitting, falls back gracefully) */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = LDR_ADC_UNIT,
        .chan     = LDR_ADC_CHANNEL,
        .atten    = LDR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali);
    if (err == ESP_OK) {
        adc_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using raw conversion");
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * TC74 — read temperature
 * ═══════════════════════════════════════════════════════════════════════ */
static esp_err_t tc74_read_temp(int8_t *out_temp)
{
    uint8_t reg = TC74_REG_TEMP;
    uint8_t raw = 0;

    esp_err_t err = i2c_master_transmit_receive(tc74_dev, &reg, 1, &raw, 1, 100);
    if (err == ESP_OK) {
        *out_temp = (int8_t)raw;
    }
    return err;
}

/* ═══════════════════════════════════════════════════════════════════════
 * TC74 — ensure normal (non-standby) mode
 * ═══════════════════════════════════════════════════════════════════════ */
static void tc74_wake(void)
{
    uint8_t cfg_reg = TC74_REG_CONFIG;
    uint8_t cfg_val = 0;

    esp_err_t err = i2c_master_transmit_receive(tc74_dev, &cfg_reg, 1, &cfg_val, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TC74: failed to read config register");
        return;
    }

    if (cfg_val & 0x80) {
        /* Sensor is in standby — clear bit 7 to wake it */
        uint8_t wake_cmd[2] = { TC74_REG_CONFIG, cfg_val & 0x7F };
        i2c_master_transmit(tc74_dev, wake_cmd, 2, 100);
        vTaskDelay(pdMS_TO_TICKS(200)); /* allow conversion to complete */
        ESP_LOGI(TAG, "TC74: woke from standby");
    } else {
        ESP_LOGI(TAG, "TC74: already in normal mode (config=0x%02X)", cfg_val);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * DHT20 — trigger + read (7-byte response)
 *
 * Protocol summary (from DHT20 datasheet):
 *   1. Send trigger command: 0xAC 0x33 0x00
 *   2. Wait ≥80 ms for measurement
 *   3. Read 7 bytes: [status, hum[19:12], hum[11:4], hum[3:0]|temp[19:16],
 *                     temp[15:8], temp[7:0], CRC]
 *   4. Check status byte bit7 == 0 (not busy)
 *   5. Decode humidity and temperature from 20-bit raw values
 * ═══════════════════════════════════════════════════════════════════════ */
static esp_err_t dht20_read(float *out_temp, float *out_hum)
{
    /* Trigger measurement */
    uint8_t trigger[3] = { 0xAC, 0x33, 0x00 };
    esp_err_t err = i2c_master_transmit(dht20_dev, trigger, 3, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DHT20: trigger failed (err=0x%x) — check SDA/SCL wiring", err);
        return err;
    }

    /* Wait for measurement (datasheet: ≥80 ms) */
    vTaskDelay(pdMS_TO_TICKS(90));

    /* Read 7 bytes */
    uint8_t data[7] = {0};
    err = i2c_master_receive(dht20_dev, data, 7, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DHT20: read failed (err=0x%x)", err);
        return err;
    }

    /* Check busy bit (bit 7 of status byte) */
    if (data[0] & 0x80) {
        ESP_LOGW(TAG, "DHT20: sensor still busy, try increasing delay");
        return ESP_ERR_TIMEOUT;
    }

    /* Decode 20-bit raw humidity */
    uint32_t raw_hum = ((uint32_t)data[1] << 12) |
                       ((uint32_t)data[2] << 4)  |
                       ((uint32_t)data[3] >> 4);

    /* Decode 20-bit raw temperature */
    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) |
                        ((uint32_t)data[4] << 8)            |
                        (uint32_t)data[5];

    *out_hum  = ((float)raw_hum  / 1048576.0f) * 100.0f;
    *out_temp = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
 * LDR — read raw ADC + voltage
 * ═══════════════════════════════════════════════════════════════════════ */
static void ldr_read(int *out_raw, int *out_mv)
{
    adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, out_raw);

    if (adc_cali_ok) {
        adc_cali_raw_to_voltage(adc_cali, *out_raw, out_mv);
    } else {
        /* Fallback: linear approximation, 12-bit, 3.3 V ref */
        *out_mv = (int)((*out_raw / 4095.0f) * 3300.0f);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Main test loop
 * ═══════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Circuit test starting ===");

    i2c_init();
    adc_init();

    /* Wake TC74 from standby if needed */
    tc74_wake();

    ESP_LOGI(TAG, "All peripherals initialised — reading every 2 s");
    ESP_LOGI(TAG, "──────────────────────────────────────────────");

    int loop = 0;
    while (1) {
        ESP_LOGI(TAG, "── Read #%d ──", ++loop);

        /* TC74 */
        int8_t tc74_temp = 0;
        esp_err_t tc74_err = tc74_read_temp(&tc74_temp);
        if (tc74_err == ESP_OK) {
            ESP_LOGI(TAG, "  TC74  temp : %d °C", tc74_temp);
        } else {
            ESP_LOGE(TAG, "  TC74  ERROR: 0x%x — check 4.7k pull-ups on SDA/SCL", tc74_err);
        }

        /* DHT20 */
        float dht_temp = 0, dht_hum = 0;
        esp_err_t dht_err = dht20_read(&dht_temp, &dht_hum);
        if (dht_err == ESP_OK) {
            ESP_LOGI(TAG, "  DHT20 temp : %.2f °C", dht_temp);
            ESP_LOGI(TAG, "  DHT20 hum  : %.2f %%", dht_hum);
        } else {
            ESP_LOGE(TAG, "  DHT20 ERROR: 0x%x — check sensor address 0x38", dht_err);
        }

        /* LDR */
        int ldr_raw = 0, ldr_mv = 0;
        ldr_read(&ldr_raw, &ldr_mv);
        ESP_LOGI(TAG, "  LDR   raw  : %d  (~%d mV)", ldr_raw, ldr_mv);

        ESP_LOGI(TAG, "──────────────────────────────────────────────");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}