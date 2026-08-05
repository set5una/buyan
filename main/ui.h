#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    UI_SENSOR_STARTING,
    UI_SENSOR_SELF_TEST,
    UI_SENSOR_GOOD,
    UI_SENSOR_ASC_ENABLED,
    UI_SENSOR_ASC_ERROR,
    UI_SENSOR_ERROR,
} ui_sensor_status_t;

/**
 * Create the static, input-free instrument screen.
 *
 * Call this after lv_init() and before the LVGL worker task starts.
 */
esp_err_t ui_create(void);

/** All functions below must be called while holding the LVGL mutex. */
void ui_set_sensor_status(ui_sensor_status_t status);
void ui_set_sensor_metrics(bool asc_on, bool self_test_ok,
                           bool temperature_offset_valid, float temperature_offset_c,
                           bool altitude_valid, uint16_t altitude_m,
                           bool data_ready, bool communication_ok);
void ui_set_countdown(uint8_t seconds_to_sample);
void ui_update_co2(uint16_t co2_ppm);
void ui_update_environment(float temperature_c, float humidity_rh);
void ui_update_chart(uint16_t co2_ppm);
