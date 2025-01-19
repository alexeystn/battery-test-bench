#include "pwm.h"
#include "driver/ledc.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

#define LEDC_OUTPUT_IO_0        (0)
#define LEDC_OUTPUT_IO_1        (1)

//#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_11_BIT
#define LEDC_FREQUENCY          2000

#define LED_PERIOD  (1<<LEDC_DUTY_RES)

void PWM_Init(void){
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = LEDC_FREQUENCY,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel_0 = {
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_0,
    .timer_sel      = LEDC_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = LEDC_OUTPUT_IO_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config_t ledc_channel_1 = {
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_1,
    .timer_sel      = LEDC_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = LEDC_OUTPUT_IO_1,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel_0);
  ledc_channel_config(&ledc_channel_1);
}

void PWM_Set(uint32_t value) {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, value);
  ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_1, value, LED_PERIOD/2);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}
