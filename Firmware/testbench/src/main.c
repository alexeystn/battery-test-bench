#include "../lib/pwm.h"
#include "../lib/adc.h"
#include "../lib/pid.h"
#include "../lib/ports.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "checker";


#define STATE_IDLE  0
#define STATE_WAIT  1
#define STATE_RUN   2
#define STATE_STOP  3

#define PWM_MAX 990

#define THRESHOLD_VOLTAGE 3.35f

#define POWER_TARGET  20.0f


uint32_t startTime = 0;
uint32_t currentTime;
uint8_t state = STATE_IDLE;

uint16_t pwmValue = 0;

float error;

float currentOffset = 0;
float offsetSumm = 0;
uint16_t offsetCounter = 0;

void app_main() {
  PWM_Init();
  ADC_Init();
  GPIO_Init();
  PID_Reset();

  LED_Write(1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  LED_Write(0);

  while(1) {
    
    float vBatt = ADC_ReadV(2) * 2;
    float vCurr = ADC_ReadV(3) * 2;

    switch (state) {
      case STATE_IDLE:
        if (Key_Read() == 0) {
          state = STATE_WAIT;
          startTime = esp_timer_get_time()/1000 + 3000;
          LED_Write(1);
          offsetSumm = 0;
          offsetCounter = 0;
        }
        break;

      case STATE_WAIT:
        pwmValue = 0;
        offsetSumm += vCurr;
        offsetCounter += 1;
        if (esp_timer_get_time()/1000 > startTime) {
          state = STATE_RUN;
          startTime = esp_timer_get_time()/1000;
          currentOffset = offsetSumm / offsetCounter;
        }
        break;
      
      case STATE_RUN:
        if (vBatt < THRESHOLD_VOLTAGE) {
          LED_Write(0);
          state = STATE_STOP;
        }
        break;
      
      case STATE_STOP:
        state = STATE_IDLE;
        break;
    }

    currentTime = esp_timer_get_time()/1000 - startTime;

    vCurr -= currentOffset;
    vCurr = vCurr * 10;  // 100 mV/A
    float wPower = vCurr * vBatt;
    float wPowerTarget = POWER_TARGET;

    #define RISE_TIME  1000
    if (currentTime < RISE_TIME) {
      wPowerTarget = POWER_TARGET * currentTime / RISE_TIME;
    }

    if (state == STATE_RUN) {
      error = wPowerTarget - wPower;
      pwmValue = PID_Process(error, currentTime) / 100.0f * PWM_MAX;
    } else {
      PID_Reset();
      pwmValue = 0;
      wPowerTarget = 0;
    }

    PWM_Set(pwmValue);

    ESP_LOGI(TAG, "%8li %8d %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f ", 
             currentTime, pwmValue, vBatt, vCurr, wPower, wPowerTarget, PID_GetP(), PID_GetI(), PID_GetD());

    vTaskDelay(50 / portTICK_PERIOD_MS);

  }

}