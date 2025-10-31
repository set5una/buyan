#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "st7701_panel.h"

#include "lvgl.h"
#include "scd4x.h"

#include "hw_layout.h"

#include "fonts.h"

#define BUF_SIZE (LCD_H_RES * LCD_V_RES * LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565)) // LVGL draw buffer size

static const char *TAG = "APP";

static i2c_dev_t scd4x_dev; // SCD4x sensor I2C handle
static bool sensor_fail;
static bool asc_enabled;

static SemaphoreHandle_t lv_sem; // LVGL Mutex

uint8_t *lvgl_dest = NULL; // Draw buffer for display rotation

static lv_obj_t *label_co2;
static lv_obj_t *label_asc;

static lv_style_t style_co2ppm;
static lv_style_t style_asc;

// Increase LVGL tick every 1ms
static void lvgl_tick_inc(void *arg)
{
    lv_tick_inc(1);
}

esp_err_t lvgl_port_lock(uint32_t timeout_ms)
{
    if (!lv_sem)
        return ESP_ERR_INVALID_STATE;

    BaseType_t ok = xSemaphoreTakeRecursive(lv_sem, timeout_ms == 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));
    return ok == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

void lvgl_port_unlock(void)
{
    if (lv_sem)
        xSemaphoreGiveRecursive(lv_sem);
}

// LVGL flush callback copied from manuf. sample code
static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    lv_area_t rotated_area;
    if (rotation != LV_DISPLAY_ROTATION_0)
    {
        lv_color_format_t cf = lv_display_get_color_format(disp);
        /*Calculate the position of the rotated area*/
        rotated_area = *area;
        lv_display_rotate_area(disp, &rotated_area);
        /*Calculate the source stride (bytes in a line) from the width of the area*/
        uint32_t src_stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
        /*Calculate the stride of the destination (rotated) area too*/
        uint32_t dest_stride = lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), cf);
        /*Have a buffer to store the rotated area and perform the rotation*/

        int32_t src_w = lv_area_get_width(area);
        int32_t src_h = lv_area_get_height(area);
        lv_draw_sw_rotate(color_p, lvgl_dest, src_w, src_h, src_stride, dest_stride, rotation, cf);
        /*Use the rotated area and rotated buffer from now on*/
        area = &rotated_area;
    }
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, lvgl_dest);
    // esp_lcd_panel_draw_bitmap(panel_handle, area-0>x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

// Wait for ISR on bounce frame finish
void lvgl_flush_wait_cb(lv_display_t *disp)
{
    st7701_wait_flush_done();
}

// Main LVGL task
static void lvgl_task(void *arg)
{
    while (1)
    {
        if (lvgl_port_lock(0) == ESP_OK)
        {
            lv_timer_handler();
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// CO2 value to text color mapping
static lv_color_t co2_color(uint16_t ppm)
{
    if (ppm < 800)
        return lv_color_white();
    if (ppm < 1300)
        return lv_color_hex(0xd77e00);
    return lv_color_hex(0xe4002b);
}

// Reads measurement from SCD4x and updates display
static void scd4x_task(void *arg)
{
    uint16_t co2_reading;
    float temp, humidity;
    bool scd4x_datardy = false;

    do
    {
        ESP_LOGI(TAG, "Waiting for SCD4x initial measurement");
        ESP_ERROR_CHECK(scd4x_get_data_ready_status(&scd4x_dev, &scd4x_datardy));
        if (!scd4x_datardy)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (!scd4x_datardy);

    lvgl_port_lock(50);
    lv_obj_add_flag(label_asc, LV_OBJ_FLAG_HIDDEN);
    lvgl_port_unlock();

    while (1)
    {
        scd4x_get_data_ready_status(&scd4x_dev, &scd4x_datardy);

        if (scd4x_datardy &&
            scd4x_read_measurement(&scd4x_dev, &co2_reading, &temp, &humidity) == ESP_OK &&
            lvgl_port_lock(100) == ESP_OK)
        {
            ESP_LOGI(TAG, "SCD4x measurement: CO2=%uppm, T=%.2fC, H=%.2f%%", co2_reading, temp, humidity);
            lv_label_set_text_fmt(label_co2, co2_reading < 1000 ? "%3u" : "%4u", co2_reading);
            // Set corresponding text color based on co2 reading
            lv_style_set_text_color(&style_co2ppm, co2_color(co2_reading));
            lv_obj_refresh_style(label_co2, LV_PART_MAIN, LV_STYLE_PROP_ANY);
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void ui_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    // lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(screen, 8, 0);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    lv_style_init(&style_co2ppm);
    lv_style_set_text_font(&style_co2ppm, &b612_200);
    lv_style_set_text_color(&style_co2ppm, lv_color_white());

    static lv_style_t style_co2txt;
    lv_style_init(&style_co2txt);
    lv_style_set_text_font(&style_co2txt, &b612_64);
    lv_style_set_text_color(&style_co2txt, lv_color_white());

    lv_style_init(&style_asc);
    lv_style_set_text_font(&style_asc, &b612_64);
    lv_style_set_text_color(&style_asc, lv_color_white());

    lv_obj_t *label_co2_txt = lv_label_create(screen);
    lv_obj_add_style(label_co2_txt, &style_co2txt, 0);
    lv_label_set_text(label_co2_txt, "CO2 PPM");
    lv_obj_set_style_align(label_co2_txt, LV_ALIGN_TOP_LEFT, 0);

    label_asc = lv_label_create(screen);
    lv_obj_add_style(label_asc, &style_asc, 0);
    lv_label_set_text(label_asc, "ASC: XX");
    lv_obj_set_style_align(label_asc, LV_ALIGN_TOP_RIGHT, 0);
    lv_style_set_text_color(&style_asc, lv_color_hex(0xd77e00));

    label_co2 = lv_label_create(screen);
    lv_obj_add_style(label_co2, &style_co2ppm, 0);
    lv_label_set_text(label_co2, "XXXX");
    lv_style_set_text_color(&style_co2ppm, lv_color_hex(0xd77e00));
    lv_obj_set_style_align(label_co2, LV_ALIGN_BOTTOM_LEFT, 0);
}

void app_main(void)
{

    esp_lcd_panel_handle_t lcd_handle;
    ESP_ERROR_CHECK(st7701_panel_init(&lcd_handle));

    lv_init();
    lv_sem = xSemaphoreCreateRecursiveMutex();
    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(disp, example_lvgl_flush_cb);
    lv_display_set_flush_wait_cb(disp, lvgl_flush_wait_cb);

    uint8_t *buf1 = (uint8_t *)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *buf2 = (uint8_t *)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM);
    lvgl_dest = (uint8_t *)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(disp, buf1, buf2, BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);
    lv_display_set_user_data(disp, lcd_handle);

    const esp_timer_create_args_t lvgl_tick_timer_cfg =
        {
            .callback = &lvgl_tick_inc,
            .name = "lvgl tick timer"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_cfg, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 1000));

    ESP_LOGI(TAG, "Creating LVGL Task");

    ui_create();
    xTaskCreatePinnedToCore(lvgl_task, "LVGL Task", 6144, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "Initializing I2C Master");
    i2cdev_init();

    ESP_ERROR_CHECK(scd4x_init_desc(&scd4x_dev, 0, I2C_SDA, I2C_SCL));
    ESP_LOGI(TAG, "Initializing SCD4x");
    // ESP_ERROR_CHECK(scd4x_wake_up(&scd4x_dev)); //DO NOT USE : FAILURE ON WARM BOOT
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&scd4x_dev));
    ESP_ERROR_CHECK(scd4x_reinit(&scd4x_dev));
    ESP_LOGI(TAG, "SCD4x initialized");
    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&scd4x_dev, serial, serial + 1, serial + 2));
    ESP_LOGI(TAG, "SCD4x SN: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    lv_label_set_text_fmt(label_asc, "SELF TEST");
    ESP_ERROR_CHECK(scd4x_perform_self_test(&scd4x_dev, &sensor_fail));
    lvgl_port_lock(50);
    lv_label_set_text_fmt(label_asc, sensor_fail ? "SENSOR FAIL" : "SENSOR GOOD");
    lv_style_set_text_color(&style_asc, sensor_fail ? lv_color_hex(0xe4002b) : lv_color_white());
    lvgl_port_unlock();
    do
    {
        vTaskDelay(pdMS_TO_TICKS(300));
    } while (sensor_fail);

    ESP_ERROR_CHECK(scd4x_set_automatic_self_calibration(&scd4x_dev, true));
    ESP_ERROR_CHECK(scd4x_get_automatic_self_calibration(&scd4x_dev, &asc_enabled));
    lvgl_port_lock(50);
    lv_label_set_text_fmt(label_asc, asc_enabled ? "ASC: ON" : "ASC: FAIL");
    lv_style_set_text_color(&style_asc, asc_enabled ? lv_color_white() : lv_color_hex(0xe4002b));
    lvgl_port_unlock();
    while (!asc_enabled)
        vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&scd4x_dev));
    ESP_LOGI(TAG, "SCD4x periodic measurements started");

    xTaskCreatePinnedToCore(scd4x_task, "SCD41 Task", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "SPIRAM FREE SIZE: %d Bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}