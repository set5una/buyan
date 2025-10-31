#include "st7701_panel.h"
#include "esp_log.h"
#include "esp_check.h"
#include "hw_layout.h"
#include "lcd_bl_pwm_bsp.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_st7701.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "ST7701 Panel";

static SemaphoreHandle_t flush_done;

static const st7701_lcd_init_cmd_t lcd_init_cmds[] =
    {
        //   cmd   data        data_size  delay_ms 1
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
        {0xEF, (uint8_t[]){0x08}, 1, 0},
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
        {0xC0, (uint8_t[]){0xE5, 0x02}, 2, 0},
        {0xC1, (uint8_t[]){0x15, 0x0A}, 2, 0},
        {0xC2, (uint8_t[]){0x07, 0x02}, 2, 0},
        {0xCC, (uint8_t[]){0x10}, 1, 0},
        {0xB0, (uint8_t[]){0x00, 0x08, 0x51, 0x0D, 0xCE, 0x06, 0x00, 0x08, 0x08, 0x24, 0x05, 0xD0, 0x0F, 0x6F, 0x36, 0x1F}, 16, 0},
        {0xB1, (uint8_t[]){0x00, 0x10, 0x4F, 0x0C, 0x11, 0x05, 0x00, 0x07, 0x07, 0x18, 0x02, 0xD3, 0x11, 0x6E, 0x34, 0x1F}, 16, 0},
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
        {0xB0, (uint8_t[]){0x4D}, 1, 0},
        {0xB1, (uint8_t[]){0x37}, 1, 0},
        {0xB2, (uint8_t[]){0x87}, 1, 0},
        {0xB3, (uint8_t[]){0x80}, 1, 0},
        {0xB5, (uint8_t[]){0x4A}, 1, 0},
        {0xB7, (uint8_t[]){0x85}, 1, 0},
        {0xB8, (uint8_t[]){0x21}, 1, 0},
        {0xB9, (uint8_t[]){0x00, 0x13}, 2, 0},
        {0xC0, (uint8_t[]){0x09}, 1, 0},
        {0xC1, (uint8_t[]){0x78}, 1, 0},
        {0xC2, (uint8_t[]){0x78}, 1, 0},
        {0xD0, (uint8_t[]){0x88}, 1, 0},
        {0xE0, (uint8_t[]){0x80, 0x00, 0x02}, 3, 100},
        {0xE1, (uint8_t[]){0x0F, 0xA0, 0x00, 0x00, 0x10, 0xA0, 0x00, 0x00, 0x00, 0x60, 0x60}, 11, 0},
        {0xE2, (uint8_t[]){0x30, 0x30, 0x60, 0x60, 0x45, 0xA0, 0x00, 0x00, 0x46, 0xA0, 0x00, 0x00, 0x00}, 13, 0},
        {0xE3, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
        {0xE4, (uint8_t[]){0x44, 0x44}, 2, 0},
        {0xE5, (uint8_t[]){0x0F, 0x4A, 0xA0, 0xA0, 0x11, 0x4A, 0xA0, 0xA0, 0x13, 0x4A, 0xA0, 0xA0, 0x15, 0x4A, 0xA0, 0xA0}, 16, 0},
        {0xE6, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
        {0xE7, (uint8_t[]){0x44, 0x44}, 2, 0},
        {0xE8, (uint8_t[]){0x10, 0x4A, 0xA0, 0xA0, 0x12, 0x4A, 0xA0, 0xA0, 0x14, 0x4A, 0xA0, 0xA0, 0x16, 0x4A, 0xA0, 0xA0}, 16, 0},
        {0xEB, (uint8_t[]){0x02, 0x00, 0x4E, 0x4E, 0xEE, 0x44, 0x00}, 7, 0},
        {0xED, (uint8_t[]){0xFF, 0xFF, 0x04, 0x56, 0x72, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x27, 0x65, 0x40, 0xFF, 0xFF}, 16, 0},
        {0xEF, (uint8_t[]){0x08, 0x08, 0x08, 0x40, 0x3F, 0x64}, 6, 0},
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
        {0xE8, (uint8_t[]){0x00, 0x0E}, 2, 0},
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
        {0x11, (uint8_t[]){0x00}, 0, 120},
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
        {0xE8, (uint8_t[]){0x00, 0x0C}, 2, 10},
        {0xE8, (uint8_t[]){0x00, 0x00}, 2, 0},
        {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
        {0x3A, (uint8_t[]){0x55}, 1, 0},
        {0x36, (uint8_t[]){0x00}, 1, 0},
        {0x35, (uint8_t[]){0x00}, 1, 0},
        {0x29, (uint8_t[]){0x00}, 0, 20},
};

static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_panel_io_handle_t s_panel_io = NULL;

static bool isr_on_bounce_frame_fin(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(flush_done, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

esp_err_t st7701_panel_init(esp_lcd_panel_handle_t *panel_handle)
{
    ESP_RETURN_ON_FALSE(panel_handle, ESP_ERR_INVALID_ARG, TAG, "panel_handle is NULL");
    *panel_handle = NULL;

    flush_done = xSemaphoreCreateBinary();

    ESP_LOGI(TAG, "Initializing LCD backlight BSP component");
    lcd_bl_pwm_bsp_init(LCD_PWM_MODE_100);

    ESP_LOGI(TAG, "Initializing LCD CMD channel SPI CFG");
    spi_line_config_t lcd_cmd_spi_conf =
        {
            .cs_io_type = IO_TYPE_GPIO,
            .cs_gpio_num = LCD_SPI_CS,
            .scl_io_type = IO_TYPE_GPIO,
            .scl_gpio_num = LCD_SPI_SCK,
            .sda_io_type = IO_TYPE_GPIO,
            .sda_gpio_num = LCD_SPI_SDO,
            .io_expander = NULL,
        };

    ESP_LOGI(TAG, "Installing LCD CMD channel SPI");
    esp_lcd_panel_io_3wire_spi_config_t lcd_spi_conf = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(lcd_cmd_spi_conf, 0);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&lcd_spi_conf, &s_panel_io));

    ESP_LOGI(TAG, "Initializing LCD panel CFG");
    esp_lcd_rgb_panel_config_t rgb_config =
        {
            .clk_src = LCD_CLK_SRC_DEFAULT,
            .psram_trans_align = 64,
            .bounce_buffer_size_px = 10 * LCD_H_RES,
            .num_fbs = 2,
            .data_width = 16,
            .bits_per_pixel = 16,
            .de_gpio_num = LCD_RGB_DE,
            .pclk_gpio_num = LCD_RGB_PCLK,
            .vsync_gpio_num = LCD_RGB_VSYNC,
            .hsync_gpio_num = LCD_RGB_HSYNC,
            .flags.fb_in_psram = true,
            .disp_gpio_num = -1,
            .data_gpio_nums =
                {
                    // BGR
                    LCD_RGB_B0,
                    LCD_RGB_B1,
                    LCD_RGB_B2,
                    LCD_RGB_B3,
                    LCD_RGB_B4,
                    LCD_RGB_G0,
                    LCD_RGB_G1,
                    LCD_RGB_G2,
                    LCD_RGB_G3,
                    LCD_RGB_G4,
                    LCD_RGB_G5,
                    LCD_RGB_R0,
                    LCD_RGB_R1,
                    LCD_RGB_R2,
                    LCD_RGB_R3,
                    LCD_RGB_R4,
                },
            .timings =
                {
                    .pclk_hz = 18 * 1000 * 1000,
                    .h_res = LCD_H_RES,
                    .v_res = LCD_V_RES,
                    .hsync_back_porch = 30,
                    .hsync_front_porch = 30, // 30
                    .hsync_pulse_width = 6,
                    .vsync_back_porch = 20,  // 10-100 40
                    .vsync_front_porch = 20, // 10-100 70
                    .vsync_pulse_width = 40,
                },
        };

    st7701_vendor_config_t vendor_config =
        {
            .rgb_config = &rgb_config,
            .init_cmds = lcd_init_cmds,
            .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
            .flags =
                {
                    .mirror_by_cmd = 1, // Only work when `enable_io_multiplex` is set to 0
                    .enable_io_multiplex = 0,
                },
        };

    const esp_lcd_panel_dev_config_t panel_config =
        {
            .reset_gpio_num = LCD_RGB_RESET,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16,
            .vendor_config = &vendor_config,
        };

    ESP_LOGI(TAG, "Installing LCD panel");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(s_panel_io, &panel_config, &s_panel));

    ESP_LOGI(TAG, "Registering LCD event callback");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_bounce_frame_finish = isr_on_bounce_frame_fin,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(s_panel, &cbs, NULL));

    ESP_LOGI(TAG, "Resetting LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));

    ESP_LOGI(TAG, "Initializing LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    *panel_handle = s_panel;
    return ESP_OK;
}

void st7701_wait_flush_done(void)
{
    xSemaphoreTake(flush_done, portMAX_DELAY);
}

esp_err_t st7701_panel_set_backlight(uint16_t duty)
{
    setUpduty(duty);
    return ESP_OK;
}
