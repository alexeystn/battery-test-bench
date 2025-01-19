#pragma once

#include <stdio.h>

void ADC_Init(void);
int16_t ADC_Read(uint8_t channel);
float ADC_ReadV(uint8_t channel);
