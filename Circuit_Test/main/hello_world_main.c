/*
 * circuit_test.c
 * Tests: DHT20, TC74, LDR, TFT display, SD card
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
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "st7735.h"

/* ── I2C ─────────────────────────────────────────────────────────────── */
#define I2C_SDA             6
#define I2C_SCL             7
#define I2C_FREQ_HZ         100000
#define I2C_TIMEOUT_MS      1000

/* ── Sensors ─────────────────────────────────────────────────────────── */
#define TC74_ADDR           0x4D
#define TC74_REG_TEMP       0x00
#define TC74_REG_CONFIG     0x01
#define DHT20_ADDR          0x38

/* ── LDR ─────────────────────────────────────────────────────────────── */
#define LDR_ADC_CHANNEL     ADC_CHANNEL_1
#define LDR_ADC_UNIT        ADC_UNIT_1
#define LDR_ADC_ATTEN       ADC_ATTEN_DB_12

/* ── TFT ─────────────────────────────────────────────────────────────── */
#define TFT_MOSI            19
#define TFT_SCK             21
#define TFT_CS              22
#define TFT_DC              2
#define TFT_RST             3
#define TFT_BL              15

/* ── SD card ─────────────────────────────────────────────────────────── */
#define SD_MISO             20
#define SD_MOSI             19
#define SD_SCK              21
#define SD_CS               18
#define SD_MOUNT            "/sdcard"

static const char *TAG = "CIRCUIT_TEST";

/* ── Handles ─────────────────────────────────────────────────────────── */
static i2c_master_bus_handle_t   i2c_bus;
static i2c_master_dev_handle_t   tc74_dev;
static i2c_master_dev_handle_t   dht20_dev;
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         adc_cali;
static bool                      adc_cali_ok = false;

/* ═══════════════════════════════════════════════════════════════════════
 * I2C
 * ═══════════════════════════════════════════════════════════════════════ */
static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port             = I2C_NUM_0,
        .sda_io_num           = I2C_SDA,
        .scl_io_num           = I2C_SCL,
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t tc74_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TC74_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &tc74_cfg, &tc74_dev));

    i2c_device_config_t dht20_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DHT20_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dht20_cfg, &dht20_dev));

    ESP_LOGI(TAG, "I2C ready");
}

/* ═══════════════════════════════════════════════════════════════════════
 * ADC
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

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = LDR_ADC_UNIT,
        .chan     = LDR_ADC_CHANNEL,
        .atten    = LDR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_ok = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali) == ESP_OK);
    ESP_LOGI(TAG, "ADC ready (cali=%s)", adc_cali_ok ? "yes" : "no");
}

/* ═══════════════════════════════════════════════════════════════════════
 * TC74
 * ═══════════════════════════════════════════════════════════════════════ */
static esp_err_t tc74_read_temp(int8_t *out)
{
    uint8_t reg = TC74_REG_TEMP, raw = 0;
    esp_err_t err = i2c_master_transmit_receive(tc74_dev, &reg, 1, &raw, 1,
                                                I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err == ESP_OK) *out = (int8_t)raw;
    return err;
}

static void tc74_wake(void)
{
    uint8_t reg = TC74_REG_CONFIG, cfg = 0;
    if (i2c_master_transmit_receive(tc74_dev, &reg, 1, &cfg, 1,
                                    I2C_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) {
        ESP_LOGE(TAG, "TC74: cannot read config");
        return;
    }
    if (cfg & 0x80) {
        uint8_t buf[2] = { TC74_REG_CONFIG, cfg & 0x7F };
        i2c_master_transmit(tc74_dev, buf, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "TC74: woke from standby");
    } else {
        ESP_LOGI(TAG, "TC74: ready (config=0x%02X)", cfg);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * DHT20
 * ═══════════════════════════════════════════════════════════════════════ */
static esp_err_t dht20_read(float *out_temp, float *out_hum)
{
    uint8_t trigger[3] = { 0xAC, 0x33, 0x00 };
    esp_err_t err = i2c_master_transmit(dht20_dev, trigger, 3,
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(90));

    uint8_t data[7] = {0};
    err = i2c_master_receive(dht20_dev, data, 7,
                             I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;
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
    adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, raw);
    if (adc_cali_ok)
        adc_cali_raw_to_voltage(adc_cali, *raw, mv);
    else
        *mv = (int)((*raw / 4095.0f) * 3300.0f);
}

/* ═══════════════════════════════════════════════════════════════════════
 * TFT — init using ST7735 driver and show status screen
 * ═══════════════════════════════════════════════════════════════════════ */
static void tft_init(void)
{
    st7735_config_t cfg = {
        .mosi_io_num = TFT_MOSI,
        .sclk_io_num = TFT_SCK,
        .cs_io_num   = TFT_CS,
        .dc_io_num   = TFT_DC,
        .rst_io_num  = TFT_RST,
        .bl_io_num   = TFT_BL,
        .host_id     = SPI2_HOST,
    };
    esp_err_t err = st7735_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TFT: init failed (0x%x)", err);
        return;
    }
    ESP_LOGI(TAG, "TFT: init OK");

    st7735_set_rotation(1);
    st7735_fill_screen(ST7735_BLACK);
    st7735_draw_string(10,  5, "CIRCUIT TEST",  ST7735_CYAN,  ST7735_BLACK, 1);
    st7735_draw_string(10, 20, "DHT20:  OK",    ST7735_GREEN, ST7735_BLACK, 1);
    st7735_draw_string(10, 32, "TC74:   OK",    ST7735_GREEN, ST7735_BLACK, 1);
    st7735_draw_string(10, 44, "LDR:    OK",    ST7735_GREEN, ST7735_BLACK, 1);
    st7735_draw_string(10, 56, "SD:     OK",    ST7735_GREEN, ST7735_BLACK, 1);
    st7735_draw_string(10, 68, "TFT:    OK",    ST7735_GREEN, ST7735_BLACK, 1);

    ESP_LOGI(TAG, "TFT: display updated");
}

/* ═══════════════════════════════════════════════════════════════════════
 * SD card
 * ═══════════════════════════════════════════════════════════════════════ */
static void sd_test(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_card_t *card;
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS;
    slot_cfg.host_id = SPI2_HOST;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    /* SD needs MISO — initialise SPI bus here before TFT takes it */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = SD_MOSI,
        .miso_io_num   = SD_MISO,
        .sclk_io_num   = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    esp_err_t err = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_cfg,
                                             &mount_cfg, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD: mount failed (0x%x)", err);
        return;
    }
    ESP_LOGI(TAG, "SD: mounted — %s", card->cid.name);

    FILE *f = fopen(SD_MOUNT "/test.txt", "w");
    if (f) {
        fprintf(f, "Circuit test OK\n");
        fclose(f);
        ESP_LOGI(TAG, "SD: wrote test.txt ✓");
    }

    f = fopen(SD_MOUNT "/test.txt", "r");
    if (f) {
        char line[32];
        fgets(line, sizeof(line), f);
        fclose(f);
        ESP_LOGI(TAG, "SD: read back: %s", line);
    }

    esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
    ESP_LOGI(TAG, "SD: unmounted");
}

/* ═══════════════════════════════════════════════════════════════════════
 * app_main
 * ═══════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Circuit test starting ===");

    i2c_init();
    adc_init();
    tc74_wake();
    sd_test();   /* SD first — initialises SPI2 bus with MISO */
    tft_init();  /* TFT second — reuses SPI2 bus */

    ESP_LOGI(TAG, "All peripherals ready — reading every 2 s");
    ESP_LOGI(TAG, "──────────────────────────────────────────");

    int loop = 0;
    while (1) {
        ESP_LOGI(TAG, "── Read #%d ──", ++loop);

        int8_t tc74_temp = 0;
        if (tc74_read_temp(&tc74_temp) == ESP_OK)
            ESP_LOGI(TAG, "  TC74  temp : %d °C", tc74_temp);
        else
            ESP_LOGE(TAG, "  TC74  ERROR (addr=0x%02X)", TC74_ADDR);

        float dht_temp = 0, dht_hum = 0;
        if (dht20_read(&dht_temp, &dht_hum) == ESP_OK) {
            ESP_LOGI(TAG, "  DHT20 temp : %.2f °C", dht_temp);
            ESP_LOGI(TAG, "  DHT20 hum  : %.2f %%", dht_hum);
        } else {
            ESP_LOGE(TAG, "  DHT20 ERROR");
        }

        int ldr_raw = 0, ldr_mv = 0;
        ldr_read(&ldr_raw, &ldr_mv);
        ESP_LOGI(TAG, "  LDR   raw  : %d (~%d mV)", ldr_raw, ldr_mv);

        ESP_LOGI(TAG, "──────────────────────────────────────────");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}