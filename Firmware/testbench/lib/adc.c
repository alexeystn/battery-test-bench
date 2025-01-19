#include "adc.h"
#include "ads1115.h"
#include <stdint.h>

void ADC_Init(void)
{
  ADS1115_Init();
}

int16_t ADC_Read(uint8_t channel)
{
  return ADS1115_Read(channel);
}

float ADC_ReadV(uint8_t channel)
{
  return ADS1115_ReadV(channel);
}