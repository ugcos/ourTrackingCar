#ifndef __CarGo_H
#define __CarGo_H

#include "main.h"

void MotorInit(void);
void CarAhead(int LeftSpeed, int RightSpeed);
void CarBack(int LeftSpeed, int RightSpeed);
void Left(int LeftSpeed, int RightSpeed);
void Right(int LeftSpeed, int RightSpeed);
void CarStop(void);
#endif
