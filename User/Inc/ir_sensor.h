#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include "stm32f1xx_hal.h"

// Car state return values:
//   0 = Stop
//   1 = Forward
//   2 = Slight left correction
//   3 = Slight right correction
//   4 = Sharp left turn
//   5 = Sharp right turn

// Sensor order (left to right): SW1(PB13), SW2(PB12), SW3(PB14), SW4(PB15)
// left1=SW1, left2=SW2, right2=SW3, right1=SW4
void IR_Read(int *left1, int *left2, int *right2, int *right1);
int IR_GetCarState(void);

#endif // __IR_SENSOR_H
