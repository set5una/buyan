#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "scd4x.h"
#include "st7701_panel.h"

#include "hw_layout.h"
#include "ui.h"

#define LVGL_DRAW_BUF_LINES 16U
#define LVGL_DRAW_BUF_SIZE \
    (LCD_V_RES * LVGL_DRAW_BUF_LINES * LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define SENSOR_SAMPLE_PERIOD_SECONDS 5U
#define SENSOR_POLL_PERIOD_MS 1000U
#define SENSOR_ERROR_REPORT_THRESHOLD 3U
#define UI_CO2_PHASE_MS 220U
#define UI_ENVIRONMENT_PHASE_MS 100U

static const char *TAG = "APP";

static i2c_dev_t scd4x_dev;
static bool sensor_fail;
static bool asc_enabled;
static bool sensor_self_test_ok;
static bool temperature_offset_valid;
static float temperature_offset_c;
static bool sensor_altitude_valid;
static uint16_t sensor_altitude_m;

static SemaphoreHandle_t lv_sem;
static uint8_t *lvgl_dest;

static void lvgl_tick_inc(void *arg)
{
    (void)arg;
    lv_tick_inc(1);
}

static esp_err_t lvgl_port_lock(uint32_t timeout_ms)
{
    if (!lv_sem)
        return ESP_ERR_INVALID_STATE;

    const BaseType_t ok = xSemaphoreTakeRecursive(
        lv_sem, timeout_ms == 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));
    return ok == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void lvgl_port_unlock(void)
{
    if (lv_sem)
        xSemaphoreGiveRecursive(lv_sem);
}

/*
 * Keep the proven single-framebuffer path.  Each small rotated band is sent
 * immediately after a fresh frame boundary, which confines any unavoidable
 * RGB-panel tear to that band instead of shifting the complete image.
 */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    const lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    lv_area_t rotated_area;
    uint8_t *draw_buffer = color_p;

    if (rotation != LV_DISPLAY_ROTATION_0)
    {
        const lv_color_format_t color_format = lv_display_get_color_format(disp);
        rotated_area = *area;
        lv_display_rotate_area(disp, &rotated_area);

        const uint32_t source_stride =
            lv_draw_buf_width_to_stride(lv_area_get_width(area), color_format);
        const uint32_t destination_stride =
            lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), color_format);
        const int32_t source_width = lv_area_get_width(area);
        const int32_t source_height = lv_area_get_height(area);

        lv_draw_sw_rotate(color_p, lvgl_dest, source_width, source_height,
                          source_stride, destination_stride, rotation, color_format);
        area = &rotated_area;
        draw_buffer = lvgl_dest;
    }

    st7701_clear_frame_completion();
    st7701_wait_frame_completion();
    const esp_err_t err = esp_lcd_panel_draw_bitmap(panel_handle,
                                                    area->x1, area->y1,
                                                    area->x2 + 1, area->y2 + 1,
                                                    draw_buffer);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Could not present RGB frame: %s", esp_err_to_name(err));
    lv_display_flush_ready(disp);
}

static void lvgl_task(void *arg)
{
    (void)arg;
    while (true)
    {
        if (lvgl_port_lock(0) == ESP_OK)
        {
            lv_timer_handler();
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void update_ui_status(ui_sensor_status_t status)
{
    if (lvgl_port_lock(0) == ESP_OK)
    {
        ui_set_sensor_status(status);
        lvgl_port_unlock();
    }
}

static void scd4x_task(void *arg)
{
    (void)arg;
    uint8_t seconds_to_sample = SENSOR_SAMPLE_PERIOD_SECONDS;
    uint8_t consecutive_errors = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_POLL_PERIOD_MS));

        bool data_ready = false;
        bool have_measurement = false;
        uint16_t co2_reading = 0;
        float temperature = 0.0f;
        float humidity = 0.0f;

        esp_err_t err = scd4x_get_data_ready_status(&scd4x_dev, &data_ready);
        if (err == ESP_OK && data_ready)
        {
            err = scd4x_read_measurement(&scd4x_dev, &co2_reading, &temperature, &humidity);
            if (err == ESP_OK)
            {
                have_measurement = true;
                seconds_to_sample = SENSOR_SAMPLE_PERIOD_SECONDS;
                consecutive_errors = 0;
                ESP_LOGI(TAG, "SCD4x measurement: CO2=%u ppm, T=%.2f C, RH=%.2f%%",
                         co2_reading, temperature, humidity);
            }
        }

        if (err != ESP_OK)
        {
            if (consecutive_errors < UINT8_MAX)
                ++consecutive_errors;
            ESP_LOGW(TAG, "SCD4x polling error: %s", esp_err_to_name(err));
        }
        else if (!have_measurement && seconds_to_sample > 0)
        {
            --seconds_to_sample;
        }

        if (have_measurement)
        {
            if (lvgl_port_lock(0) == ESP_OK)
            {
                ui_update_co2(co2_reading);
                lvgl_port_unlock();
            }

            vTaskDelay(pdMS_TO_TICKS(UI_CO2_PHASE_MS));
            if (lvgl_port_lock(0) == ESP_OK)
            {
                ui_update_environment(temperature, humidity);
                lvgl_port_unlock();
            }

            vTaskDelay(pdMS_TO_TICKS(UI_ENVIRONMENT_PHASE_MS));
            if (lvgl_port_lock(0) == ESP_OK)
            {
                ui_update_chart(co2_reading);
                ui_set_sensor_metrics(asc_enabled, sensor_self_test_ok,
                                      temperature_offset_valid, temperature_offset_c,
                                      sensor_altitude_valid, sensor_altitude_m,
                                      data_ready,
                                      consecutive_errors < SENSOR_ERROR_REPORT_THRESHOLD);
                ui_set_countdown(seconds_to_sample);
                lvgl_port_unlock();
            }
        }
        else if (lvgl_port_lock(0) == ESP_OK)
        {
            ui_set_sensor_metrics(asc_enabled, sensor_self_test_ok,
                                  temperature_offset_valid, temperature_offset_c,
                                  sensor_altitude_valid, sensor_altitude_m,
                                  data_ready,
                                  consecutive_errors < SENSOR_ERROR_REPORT_THRESHOLD);
            ui_set_countdown(seconds_to_sample);
            lvgl_port_unlock();
        }
    }
}

void app_main(void)
{
    esp_lcd_panel_handle_t lcd_handle;
    ESP_ERROR_CHECK(st7701_panel_init(&lcd_handle));

    lv_init();
    lv_sem = xSemaphoreCreateRecursiveMutex();
    ESP_ERROR_CHECK(lv_sem ? ESP_OK : ESP_ERR_NO_MEM);

    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);
    ESP_ERROR_CHECK(display ? ESP_OK : ESP_ERR_NO_MEM);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    uint8_t *buffer_1 = heap_caps_malloc(LVGL_DRAW_BUF_SIZE,
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t *buffer_2 = heap_caps_malloc(LVGL_DRAW_BUF_SIZE,
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    lvgl_dest = heap_caps_malloc(LVGL_DRAW_BUF_SIZE,
                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(buffer_1 && buffer_2 && lvgl_dest ? ESP_OK : ESP_ERR_NO_MEM);

    lv_display_set_buffers(display, buffer_1, buffer_2, LVGL_DRAW_BUF_SIZE,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_rotation(display, LV_DISPLAY_ROTATION_270);
    lv_display_set_user_data(display, lcd_handle);

    const esp_timer_create_args_t lvgl_tick_timer_config = {
        .callback = lvgl_tick_inc,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_config, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 1000));

    ESP_ERROR_CHECK(ui_create());
    ESP_LOGI(TAG, "Starting LVGL task");
    xTaskCreatePinnedToCore(lvgl_task, "LVGL", 6144, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "Initializing I2C master and SCD4x");
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(scd4x_init_desc(&scd4x_dev, 0, I2C_SDA, I2C_SCL));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&scd4x_dev));
    ESP_ERROR_CHECK(scd4x_reinit(&scd4x_dev));

    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&scd4x_dev, serial, serial + 1, serial + 2));
    ESP_LOGI(TAG, "SCD4x serial: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    update_ui_status(UI_SENSOR_SELF_TEST);
    ESP_ERROR_CHECK(scd4x_perform_self_test(&scd4x_dev, &sensor_fail));
    if (sensor_fail)
    {
        ESP_LOGE(TAG, "SCD4x self-test reported a malfunction");
        update_ui_status(UI_SENSOR_ERROR);
        while (true)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    sensor_self_test_ok = true;

    esp_err_t offset_err = scd4x_get_temperature_offset(&scd4x_dev, &temperature_offset_c);
    temperature_offset_valid = offset_err == ESP_OK;
    if (!temperature_offset_valid)
        ESP_LOGW(TAG, "Could not read SCD4x temperature offset: %s", esp_err_to_name(offset_err));
    else
        ESP_LOGI(TAG, "SCD4x temperature offset: %.2f C", temperature_offset_c);

    esp_err_t altitude_err = scd4x_get_sensor_altitude(&scd4x_dev, &sensor_altitude_m);
    sensor_altitude_valid = altitude_err == ESP_OK;
    if (!sensor_altitude_valid)
        ESP_LOGW(TAG, "Could not read SCD4x altitude: %s", esp_err_to_name(altitude_err));
    else
        ESP_LOGI(TAG, "SCD4x altitude compensation: %u m", sensor_altitude_m);

    update_ui_status(UI_SENSOR_GOOD);
    vTaskDelay(pdMS_TO_TICKS(600));

    asc_enabled = false;
    esp_err_t asc_err = scd4x_set_automatic_self_calibration(&scd4x_dev, true);
    if (asc_err == ESP_OK)
        asc_err = scd4x_get_automatic_self_calibration(&scd4x_dev, &asc_enabled);

    if (asc_err != ESP_OK || !asc_enabled)
    {
        ESP_LOGW(TAG, "SCD4x ASC could not be enabled: %s", esp_err_to_name(asc_err));
        update_ui_status(UI_SENSOR_ASC_ERROR);
    }
    else
    {
        ESP_LOGI(TAG, "SCD4x automatic self-calibration enabled");
        update_ui_status(UI_SENSOR_ASC_ENABLED);
    }

    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&scd4x_dev));
    ESP_LOGI(TAG, "SCD4x periodic measurements started");

    if (lvgl_port_lock(0) == ESP_OK)
    {
        ui_set_sensor_metrics(asc_enabled, sensor_self_test_ok,
                              temperature_offset_valid, temperature_offset_c,
                              sensor_altitude_valid, sensor_altitude_m,
                              false, true);
        ui_set_countdown(SENSOR_SAMPLE_PERIOD_SECONDS);
        lvgl_port_unlock();
    }
    xTaskCreatePinnedToCore(scd4x_task, "SCD41", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Free PSRAM: %u bytes",
             (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    while (true)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
