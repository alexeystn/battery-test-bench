#pragma once

#include <stdio.h>

void PID_Reset();
float PID_Process(float error, uint32_t time);

float PID_GetP(void);
float PID_GetI(void);
float PID_GetD(void);
