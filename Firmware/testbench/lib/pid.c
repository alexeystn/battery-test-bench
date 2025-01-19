#include "pid.h"

#define P_COEF   0.2f
#define I_COEF   0.02f
#define D_COEF   -20.0f

float iSum = 0;
uint32_t timePrev = 0;

float iComp;
float pComp;
float dComp;


void PID_Reset(void) {
  iSum = 0;
  timePrev = 0;
  pComp = 0;
  iComp = 0;
  dComp = 0;
}


float PID_Process(float error, uint32_t time) {
  // returns 0..100%

  uint32_t timeDelta;
  float result;

  if (timePrev == 0) {
    timeDelta = 0;
  } else {
    timeDelta = time - timePrev;
  }
  timePrev = time;

  pComp = error * P_COEF;
  iComp += error * timeDelta * I_COEF;
  if (timeDelta != 0) {
    dComp = error * D_COEF / timeDelta;
  } else {
    dComp = 0;
  }

  result = iComp + pComp + dComp;

  if (result < 0) {
    result = 0;
  }
  if (result > 100) {
    result = 100;
  }
  return result;
}

float PID_GetP(void) {
  return pComp;
}

float PID_GetI(void) {
  return iComp;
}

float PID_GetD(void) {
  return dComp;
}