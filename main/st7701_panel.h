#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t st7701_panel_init(esp_lcd_panel_handle_t *panel_handle);

void st7701_clear_frame_completion(void);
void st7701_wait_frame_completion(void);

esp_err_t st7701_panel_set_backlight(uint16_t duty);

#ifdef __cplusplus
}
#endif
