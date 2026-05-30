/**
 * @file main.c
 * @brief Multi-Room Monitoring Agent — entry point.
 *
 * Initialises all peripherals and spawns FreeRTOS tasks.
 * Change ROOM_ID in shared_data.h before flashing each board.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "st7735.h"
#include "shared_data.h"

/* ── Task includes ───────────────────────────────────────────────────── */
#include "tasks/sensor_task.h"
#include "tasks/display_task.h"
#include "tasks/mqtt_task.h"
#include "tasks/sd_task.h"

static const char *TAG = "MAIN";

/* ── Pin definitions ─────────────────────────────────────────────────── */
#define I2C_SDA         6
#define I2C_SCL         7
#define I2C_FREQ_HZ     100000

#define LDR_ADC_CHANNEL ADC_CHANNEL_1
#define LDR_ADC_UNIT    ADC_UNIT_1
#define LDR_ADC_ATTEN   ADC_ATTEN_DB_12

#define TFT_MOSI        19
#define TFT_SCK         21
#define TFT_CS          22
#define TFT_DC          2
#define TFT_RST         3
#define TFT_BL          15

#define SD_MISO         20
#define SD_MOSI         19
#define SD_SCK          21
#define SD_CS           18
#define SD_MOUNT        "/sdcard"

/* ── Sensor addresses ────────────────────────────────────────────────── */
#define TC74_ADDR       0x4D
#define DHT20_ADDR      0x38

/* ── Globals ─────────────────────────────────────────────────────────── */
sensor_data_t    g_sensor_data = {0};
SemaphoreHandle_t g_data_mutex  = NULL;

/* Peripheral handles — shared with tasks via extern in task headers */
i2c_master_bus_handle_t  g_i2c_bus;
i2c_master_dev_handle_t  g_tc74_dev;
i2c_master_dev_handle_t  g_dht20_dev;
adc_oneshot_unit_handle_t g_adc_handle;
adc_cali_handle_t         g_adc_cali;
bool                      g_adc_cali_ok = false;

/* ═══════════════════════════════════════════════════════════════════════
 * I2C init
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
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &g_i2c_bus));

    i2c_device_config_t tc74_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TC74_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus, &tc74_cfg, &g_tc74_dev));

    i2c_device_config_t dht20_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DHT20_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus, &dht20_cfg, &g_dht20_dev));

    ESP_LOGI(TAG, "I2C ready");
}

/* ═══════════════════════════════════════════════════════════════════════
 * ADC init
 * ═══════════════════════════════════════════════════════════════════════ */
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = LDR_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = LDR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, LDR_ADC_CHANNEL, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = LDR_ADC_UNIT,
        .chan     = LDR_ADC_CHANNEL,
        .atten    = LDR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    g_adc_cali_ok = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &g_adc_cali) == ESP_OK);
    ESP_LOGI(TAG, "ADC ready (cali=%s)", g_adc_cali_ok ? "yes" : "no");
}

/* ═══════════════════════════════════════════════════════════════════════
 * SPI + SD init
 * ═══════════════════════════════════════════════════════════════════════ */
static void sd_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = SD_MOSI,
        .miso_io_num   = SD_MISO,
        .sclk_io_num   = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

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

    esp_err_t err = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_cfg,
                                             &mount_cfg, &card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD: mount failed (0x%x) — SD logging disabled", err);
    } else {
        ESP_LOGI(TAG, "SD: mounted — %s", card->cid.name);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * TFT init
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
    st7735_set_rotation(1);
    st7735_fill_screen(ST7735_BLACK);
    st7735_draw_string(20, 30, "ROOM " , ST7735_CYAN, ST7735_BLACK, 2);

    char room_str[2];
    snprintf(room_str, sizeof(room_str), "%d", ROOM_ID);
    st7735_draw_string(90, 30, room_str, ST7735_CYAN, ST7735_BLACK, 2);
    st7735_draw_string(15, 55, "Starting...", ST7735_GRAY, ST7735_BLACK, 1);

    ESP_LOGI(TAG, "TFT: ready");
}

/* ═══════════════════════════════════════════════════════════════════════
 * app_main
 * ═══════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Multi-Room Monitoring Agent ===");
    ESP_LOGI(TAG, "Room ID: %d", ROOM_ID);

    /* Mutex */
    g_data_mutex = xSemaphoreCreateMutex();
    assert(g_data_mutex != NULL);

    /* Hardware init */
    i2c_init();
    adc_init();
    sd_init();
    tft_init();

    /* Spawn tasks */
    xTaskCreate(sensor_task,  "sensor",  4096, NULL, 3, NULL);
    xTaskCreate(display_task, "display", 4096, NULL, 2, NULL);
    xTaskCreate(mqtt_task,    "mqtt",    8192, NULL, 2, NULL);
    xTaskCreate(sd_task,      "sd",      4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "All tasks started");
}