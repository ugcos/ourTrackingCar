#include "CarGo.h"
#include "stm32f1xx_hal.h"
#include "tim.h"



void MotorInit(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}

void CarAhead(int LeftSpeed, int RightSpeed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,LeftSpeed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,RightSpeed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);


}

void CarStop(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
}



void CarBack(int LeftSpeed,int RightSpeed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,LeftSpeed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,RightSpeed);
}



void Left(int LeftSpeed,int RightSpeed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,LeftSpeed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,RightSpeed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
}



void Right(int LeftSpeed,int RightSpeed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,LeftSpeed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,RightSpeed);
}






