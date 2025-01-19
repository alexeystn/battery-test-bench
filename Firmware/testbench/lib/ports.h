#pragma once

#include <stdio.h>

void GPIO_Init();

void LED_Write(uint8_t state);
uint8_t Key_Read(void);

