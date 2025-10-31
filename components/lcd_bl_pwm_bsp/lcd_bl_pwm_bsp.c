#include <stdio.h>
#include "lcd_bl_pwm_bsp.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define EXAMPLE_PIN_NUM_BK_LIGHT   6

void lcd_bl_pwm_bsp_init(uint16_t duty)
{ 
  ledc_timer_config_t timer_conf = 
  {
    .speed_mode =  LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT, //256
    .timer_num =  LEDC_TIMER_3,
    .freq_hz = 50 * 1000,
    .clk_cfg = LEDC_SLOW_CLK_RC_FAST,
  };
  ledc_channel_config_t ledc_conf = 
  {
    .gpio_num = EXAMPLE_PIN_NUM_BK_LIGHT,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel =  LEDC_CHANNEL_1,
    .intr_type =  LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_3,
    .duty = duty,   //Duty Cycle
    .hpoint = 0,    //Phase
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&timer_conf));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&ledc_conf));
}

void setUpduty(uint16_t duty)
{
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}