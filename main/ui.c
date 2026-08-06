#include "ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "lvgl.h"

#include "fonts.h"

#define COLOR_AMBER 0xd77e00
#define COLOR_RED 0xe4002b
#define COLOR_GREEN 0x00c853
#define COLOR_AXIS 0x676767
#define COLOR_DIM 0x333333
#define COLOR_FOOTER_TEXT 0xa6a6a6
#define COLOR_FOOTER_LINE 0x666666

#define SENSOR_SAMPLE_PERIOD_SECONDS 5U
#define HISTORY_DURATION_SECONDS (6U * 60U * 60U)
#define HISTORY_CAPACITY (HISTORY_DURATION_SECONDS / SENSOR_SAMPLE_PERIOD_SECONDS)

#define CHART_X 438
#define CHART_Y 111
#define CHART_WIDTH 362
#define CHART_HEIGHT 148
#define PLOT_LEFT 8
#define PLOT_RIGHT 359
#define PLOT_TOP 4
#define PLOT_BOTTOM 140
#define PLOT_POINT_COUNT (PLOT_RIGHT - PLOT_LEFT + 1)

#define GAUGE_SEGMENTS 24
#define GAUGE_SAFE_SEGMENTS 8
#define GAUGE_CAUTION_SEGMENTS 8
#define GAUGE_MIN_PPM 400U
#define GAUGE_MAX_PPM 2000U

static lv_obj_t *label_co2;
static lv_obj_t *label_temp;
static lv_obj_t *label_humidity;
static lv_obj_t *label_sensor_metrics;
static lv_obj_t *label_countdown;
static lv_obj_t *status_led;
static lv_obj_t *chart_canvas;
static lv_obj_t *gauge_segments[GAUGE_SEGMENTS];
static uint32_t gauge_active_index;
static bool gauge_initialized;

static uint8_t *chart_buffer;
static uint32_t chart_buffer_stride;
static size_t chart_buffer_size;
static uint16_t co2_history[HISTORY_CAPACITY];
static uint32_t history_head;
static uint32_t history_count;
static uint64_t history_sum;

static uint16_t chart_values[PLOT_POINT_COUNT];
static bool chart_value_valid[PLOT_POINT_COUNT];
static int16_t chart_y[PLOT_POINT_COUNT];

static lv_obj_t *create_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color,
                              int32_t x, int32_t y, const char *text)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_label_set_text(label, text);
    return label;
}

static lv_obj_t *create_rect(lv_obj_t *parent, int32_t x, int32_t y, int32_t width, int32_t height,
                             lv_color_t color)
{
    lv_obj_t *rect = lv_obj_create(parent);
    lv_obj_set_pos(rect, x, y);
    lv_obj_set_size(rect, width, height);
    lv_obj_remove_flag(rect, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(rect, 0, 0);
    lv_obj_set_style_radius(rect, 0, 0);
    lv_obj_set_style_border_width(rect, 0, 0);
    lv_obj_set_style_bg_color(rect, color, 0);
    lv_obj_set_style_bg_opa(rect, LV_OPA_COVER, 0);
    return rect;
}

static void set_label_tenths(lv_obj_t *label, float value, const char *suffix)
{
    const int32_t tenths = (int32_t)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
    const bool negative = tenths < 0;
    const uint32_t magnitude = (uint32_t)(negative ? -tenths : tenths);

    lv_label_set_text_fmt(label, "%s%lu.%lu%s", negative ? "-" : "",
                          (unsigned long)(magnitude / 10U),
                          (unsigned long)(magnitude % 10U), suffix);
}

static lv_color_t gauge_color(uint32_t index)
{
    if (index < GAUGE_SAFE_SEGMENTS)
        return lv_color_white();
    if (index < GAUGE_SAFE_SEGMENTS + GAUGE_CAUTION_SEGMENTS)
        return lv_color_hex(COLOR_AMBER);
    return lv_color_hex(COLOR_RED);
}

static uint32_t gauge_index_for_ppm(uint16_t ppm)
{
    if (ppm <= GAUGE_MIN_PPM)
        return 0;
    if (ppm >= GAUGE_MAX_PPM)
        return GAUGE_SEGMENTS - 1;

    return ((uint32_t)(ppm - GAUGE_MIN_PPM) * (GAUGE_SEGMENTS - 1)) /
           (GAUGE_MAX_PPM - GAUGE_MIN_PPM);
}

static void update_gauge(uint16_t ppm)
{
    const uint32_t active = gauge_index_for_ppm(ppm);

    if (!gauge_initialized)
    {
        for (uint32_t i = 0; i < GAUGE_SEGMENTS; ++i)
            lv_obj_set_style_bg_opa(gauge_segments[i], i <= active ? LV_OPA_COVER : LV_OPA_30, 0);
        gauge_initialized = true;
    }
    else if (active > gauge_active_index)
    {
        for (uint32_t i = gauge_active_index + 1U; i <= active; ++i)
            lv_obj_set_style_bg_opa(gauge_segments[i], LV_OPA_COVER, 0);
    }
    else if (active < gauge_active_index)
    {
        for (uint32_t i = active + 1U; i <= gauge_active_index; ++i)
            lv_obj_set_style_bg_opa(gauge_segments[i], LV_OPA_30, 0);
    }

    gauge_active_index = active;
}

static void history_push(uint16_t ppm)
{
    if (history_count == HISTORY_CAPACITY)
    {
        history_sum -= co2_history[history_head];
    }
    else
    {
        ++history_count;
    }

    co2_history[history_head] = ppm;
    history_sum += ppm;
    history_head = (history_head + 1U) % HISTORY_CAPACITY;
}

static uint16_t history_get(uint32_t chronological_index)
{
    const uint32_t oldest = (history_head + HISTORY_CAPACITY - history_count) % HISTORY_CAPACITY;
    return co2_history[(oldest + chronological_index) % HISTORY_CAPACITY];
}

static void chart_set_px(int32_t x, int32_t y, lv_color_t color)
{
    if (x < 0 || x >= CHART_WIDTH || y < 0 || y >= CHART_HEIGHT)
        return;
    uint16_t *pixel = (uint16_t *)(chart_buffer + (size_t)y * chart_buffer_stride +
                                   (size_t)x * sizeof(uint16_t));
    *pixel = lv_color_to_u16(color);
}

static void chart_draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, lv_color_t color)
{
    const int32_t dx = abs(x1 - x0);
    const int32_t sx = x0 < x1 ? 1 : -1;
    const int32_t dy = -abs(y1 - y0);
    const int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;

    while (true)
    {
        chart_set_px(x0, y0, color);
        if (x0 == x1 && y0 == y1)
            break;
        const int32_t e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static int16_t value_to_chart_y(uint16_t value, uint16_t minimum, uint16_t maximum)
{
    const uint32_t range = maximum - minimum;
    const uint32_t scaled = ((uint32_t)(value - minimum) * (PLOT_BOTTOM - PLOT_TOP - 1)) / range;
    return (int16_t)(PLOT_BOTTOM - 1 - scaled);
}

static void build_chart_values(void)
{
    for (uint32_t x = 0; x < PLOT_POINT_COUNT; ++x)
    {
        chart_value_valid[x] = false;
        chart_values[x] = 0;
    }

    if (history_count == 0)
        return;

    if (history_count <= PLOT_POINT_COUNT)
    {
        const uint32_t offset = PLOT_POINT_COUNT - history_count;
        for (uint32_t i = 0; i < history_count; ++i)
        {
            chart_values[offset + i] = history_get(i);
            chart_value_valid[offset + i] = true;
        }
        return;
    }

    for (uint32_t x = 0; x < PLOT_POINT_COUNT; ++x)
    {
        uint32_t first = (x * history_count) / PLOT_POINT_COUNT;
        uint32_t end = ((x + 1U) * history_count) / PLOT_POINT_COUNT;
        if (end <= first)
            end = first + 1U;

        uint32_t sum = 0;
        for (uint32_t i = first; i < end; ++i)
            sum += history_get(i);

        chart_values[x] = (uint16_t)(sum / (end - first));
        chart_value_valid[x] = true;
    }
}

static void render_chart(void)
{
    lv_display_t *display = lv_obj_get_display(chart_canvas);
    lv_display_enable_invalidation(display, false);
    memset(chart_buffer, 0, chart_buffer_size);
    build_chart_values();

    bool have_points = false;
    uint16_t minimum = UINT16_MAX;
    uint16_t maximum = 0;
    for (uint32_t i = 0; i < PLOT_POINT_COUNT; ++i)
    {
        if (!chart_value_valid[i])
            continue;
        have_points = true;
        if (chart_values[i] < minimum)
            minimum = chart_values[i];
        if (chart_values[i] > maximum)
            maximum = chart_values[i];
    }

    uint16_t average = 0;
    if (have_points)
    {
        average = (uint16_t)(history_sum / history_count);
        uint32_t low = minimum;
        uint32_t high = maximum;
        if (high - low < 200U)
        {
            low = average > 100U ? average - 100U : 0U;
            high = average + 100U;
        }
        else
        {
            const uint32_t padding = (high - low) / 10U;
            low = low > padding ? low - padding : 0U;
            high += padding;
        }
        if (high <= low)
            high = low + 1U;

        minimum = (uint16_t)low;
        maximum = high > UINT16_MAX ? UINT16_MAX : (uint16_t)high;

        for (uint32_t i = 0; i < PLOT_POINT_COUNT; ++i)
        {
            if (chart_value_valid[i])
                chart_y[i] = value_to_chart_y(chart_values[i], minimum, maximum);
        }

        for (int32_t y = PLOT_TOP; y < PLOT_BOTTOM; ++y)
        {
            for (uint32_t i = 0; i < PLOT_POINT_COUNT; ++i)
            {
                if (!chart_value_valid[i] || y <= chart_y[i])
                    continue;
                const int32_t depth = PLOT_BOTTOM - chart_y[i];
                const uint8_t shade = (uint8_t)(8 + ((PLOT_BOTTOM - y) * 66) / depth);
                chart_set_px(PLOT_LEFT + i, y, lv_color_make(shade, shade, shade));
            }
        }
    }

    const lv_color_t axis = lv_color_hex(COLOR_AXIS);
    chart_draw_line(PLOT_LEFT, PLOT_TOP, PLOT_LEFT, PLOT_BOTTOM, axis);
    chart_draw_line(PLOT_LEFT, PLOT_BOTTOM, PLOT_RIGHT, PLOT_BOTTOM, axis);

    for (uint32_t tick = 0; tick <= 6; ++tick)
    {
        const int32_t x = PLOT_LEFT + (tick * (PLOT_RIGHT - PLOT_LEFT)) / 6;
        chart_draw_line(x, PLOT_BOTTOM, x, PLOT_BOTTOM - 5, axis);
    }
    for (uint32_t tick = 1; tick <= 4; ++tick)
    {
        const int32_t y = PLOT_TOP + (tick * (PLOT_BOTTOM - PLOT_TOP)) / 5;
        chart_draw_line(PLOT_LEFT, y, PLOT_LEFT + 5, y, axis);
    }

    int16_t average_y = PLOT_BOTTOM;
    if (have_points)
    {
        average_y = value_to_chart_y(average, minimum, maximum);
        const lv_color_t average_color = lv_color_hex(0xa0a0a0);
        for (int32_t x = PLOT_LEFT; x <= PLOT_RIGHT; x += 10)
            chart_draw_line(x, average_y, x + 5 < PLOT_RIGHT ? x + 5 : PLOT_RIGHT, average_y, average_color);

        const lv_color_t line_color = lv_color_white();
        int32_t previous = -1;
        for (uint32_t i = 0; i < PLOT_POINT_COUNT; ++i)
        {
            if (!chart_value_valid[i])
                continue;
            if (previous >= 0)
            {
                chart_draw_line(PLOT_LEFT + previous, chart_y[previous], PLOT_LEFT + i, chart_y[i], line_color);
                chart_draw_line(PLOT_LEFT + previous, chart_y[previous] + 1, PLOT_LEFT + i, chart_y[i] + 1,
                                line_color);
            }
            previous = (int32_t)i;
        }
    }

    lv_display_enable_invalidation(display, true);
    lv_obj_invalidate(chart_canvas);

}

esp_err_t ui_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(screen, 0, 0);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    create_label(screen, &b612_32, lv_color_white(), 8, 2, "CO2");
    // create_label(screen, &b612_20, lv_color_white(), 94, 11, "PPM");
    label_co2 = create_label(screen, &co2_light_172, lv_color_white(), 6, 58, "----");
    lv_obj_set_width(label_co2, 420);
    lv_obj_set_style_text_letter_space(label_co2, 0, 0);

    create_label(screen, &b612_20, lv_color_white(), 438, 8, "TEMP");
    label_temp = create_label(screen, &b612_40, lv_color_white(), 438, 36, "--.-°C");
    create_label(screen, &b612_20, lv_color_white(), 650, 8, "RH");
    label_humidity = create_label(screen, &b612_40, lv_color_white(), 650, 36, "--%");

    chart_buffer_stride = lv_draw_buf_width_to_stride(CHART_WIDTH, LV_COLOR_FORMAT_RGB565);
    chart_buffer_size = chart_buffer_stride * CHART_HEIGHT;
    chart_buffer = heap_caps_malloc(chart_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!chart_buffer)
        chart_buffer = heap_caps_malloc(chart_buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!chart_buffer)
        return ESP_ERR_NO_MEM;

    chart_canvas = lv_canvas_create(screen);
    lv_canvas_set_buffer(chart_canvas, chart_buffer, CHART_WIDTH, CHART_HEIGHT, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(chart_canvas, CHART_X, CHART_Y);

    for (uint32_t i = 0; i < GAUGE_SEGMENTS; ++i)
    {
        gauge_segments[i] = create_rect(screen, 8 + i * 17, 224, 14, 16, gauge_color(i));
        lv_obj_set_style_bg_opa(gauge_segments[i], LV_OPA_30, 0);
    }

    create_rect(screen, 8, 278, 792, 1, lv_color_hex(COLOR_FOOTER_LINE));
    status_led = create_rect(screen, 8, 295, 11, 11, lv_color_hex(COLOR_AMBER));
    label_sensor_metrics = create_label(screen, &b612_20, lv_color_hex(COLOR_FOOTER_TEXT),
                                        30, 288, "SCD41 STARTING");
    lv_obj_set_width(label_sensor_metrics, 690);
    lv_label_set_long_mode(label_sensor_metrics, LV_LABEL_LONG_CLIP);
    label_countdown = create_label(screen, &b612_20, lv_color_hex(COLOR_FOOTER_TEXT),
                                   734, 288, "-- s");
    lv_obj_set_width(label_countdown, 66);
    lv_obj_set_style_text_align(label_countdown, LV_TEXT_ALIGN_RIGHT, 0);

    update_gauge(GAUGE_MIN_PPM);
    render_chart();
    return ESP_OK;
}

void ui_set_sensor_status(ui_sensor_status_t status)
{
    const char *text = "SCD41 STARTING";
    lv_color_t color = lv_color_hex(COLOR_AMBER);

    switch (status)
    {
    case UI_SENSOR_SELF_TEST:
        text = "SCD41 SELF TEST";
        break;
    case UI_SENSOR_GOOD:
        text = "SCD41 TEST OK";
        color = lv_color_hex(COLOR_GREEN);
        break;
    case UI_SENSOR_ASC_ENABLED:
        text = "";
        color = lv_color_hex(COLOR_GREEN);
        break;
    case UI_SENSOR_ASC_ERROR:
        text = "ASC ERROR";
        color = lv_color_hex(COLOR_RED);
        break;
    case UI_SENSOR_ERROR:
        text = "SCD41 ERROR";
        color = lv_color_hex(COLOR_RED);
        break;
    case UI_SENSOR_STARTING:
    default:
        break;
    }

    lv_label_set_text(label_sensor_metrics, text);
    lv_obj_set_style_bg_color(status_led, color, 0);
}

void ui_set_sensor_metrics(bool asc_on, bool self_test_ok,
                           bool temperature_offset_valid, float temperature_offset_c,
                           bool altitude_valid, uint16_t altitude_m,
                           bool data_ready, bool communication_ok)
{
    char offset_text[16] = "--.-C";
    char altitude_text[16] = "---M";

    if (temperature_offset_valid)
    {
        const int32_t tenths = (int32_t)(temperature_offset_c * 10.0f +
                                         (temperature_offset_c >= 0.0f ? 0.5f : -0.5f));
        const bool negative = tenths < 0;
        const uint32_t magnitude = (uint32_t)(negative ? -tenths : tenths);
        snprintf(offset_text, sizeof(offset_text), "%s%lu.%luC", negative ? "-" : "",
                 (unsigned long)(magnitude / 10U), (unsigned long)(magnitude % 10U));
    }

    if (altitude_valid)
        snprintf(altitude_text, sizeof(altitude_text), "%uM", (unsigned int)altitude_m);

    const char *live_state = !communication_ok ? "COM ERR" : (data_ready ? "RDY" : "WAIT");
    /*
    lv_label_set_text_fmt(label_sensor_metrics,
                          "ASC %s   TEST %s   T-OFF %s   ALT %s   %s",
                          asc_on ? "ON" : "OFF", self_test_ok ? "OK" : "FAIL",
                          offset_text, altitude_text, live_state);
    */
    const lv_color_t color = !communication_ok || !self_test_ok
                                 ? lv_color_hex(COLOR_RED)
                                 : (asc_on ? lv_color_hex(COLOR_GREEN) : lv_color_hex(COLOR_AMBER));
    lv_obj_set_style_bg_color(status_led, color, 0);
}

void ui_set_countdown(uint8_t seconds_to_sample)
{
    if (seconds_to_sample > 99U)
        seconds_to_sample = 99U;
    lv_label_set_text_fmt(label_countdown, "%02u s", seconds_to_sample);
}

void ui_update_co2(uint16_t co2_ppm)
{
    lv_label_set_text_fmt(label_co2, "%u", co2_ppm);
    update_gauge(co2_ppm);
}

void ui_update_environment(float temperature_c, float humidity_rh)
{
    set_label_tenths(label_temp, temperature_c, "°C");
    set_label_tenths(label_humidity, humidity_rh, "%");
}

void ui_update_chart(uint16_t co2_ppm)
{
    history_push(co2_ppm);
    render_chart();
}
