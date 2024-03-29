#include "motor.h"
#include "pid.h"
#include "jy62.h"

MOTOR_Typedef motor[5];
float motor_speed_x, motor_speed_y;
int PWM[5];

void MOTOR_Direction(Turn d, uint8_t index, int16_t pwm)
{
	motor[index].dic = d;
	switch (index)
	{
		case 1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
			if (d == positive)
			{
				HAL_GPIO_WritePin(in1_1_GPIO_Port, in1_1_Pin, SET);
				HAL_GPIO_WritePin(in1_2_GPIO_Port, in1_2_Pin, RESET);
			}
			else if (d == negative)
			{
				HAL_GPIO_WritePin(in1_1_GPIO_Port, in1_1_Pin, RESET);
				HAL_GPIO_WritePin(in1_2_GPIO_Port, in1_2_Pin, SET);

			}
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
			if (d == positive)
			{
				HAL_GPIO_WritePin(in2_1_GPIO_Port, in2_1_Pin, RESET);
				HAL_GPIO_WritePin(in2_2_GPIO_Port, in2_2_Pin, SET);

			}
			else if (d == negative)
			{
				HAL_GPIO_WritePin(in2_1_GPIO_Port, in2_1_Pin, SET);
				HAL_GPIO_WritePin(in2_2_GPIO_Port, in2_2_Pin, RESET);
			}
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
			if (d == positive)
			{
				HAL_GPIO_WritePin(in3_1_GPIO_Port, in3_1_Pin, SET);
				HAL_GPIO_WritePin(in3_2_GPIO_Port, in3_2_Pin, RESET);
			}
			else if (d == negative)
			{
				HAL_GPIO_WritePin(in3_1_GPIO_Port, in3_1_Pin, RESET);
				HAL_GPIO_WritePin(in3_2_GPIO_Port, in3_2_Pin, SET);
			}
			break;
		case 4:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
			if (d == positive)
			{
				HAL_GPIO_WritePin(in4_1_GPIO_Port, in4_1_Pin, RESET);
				HAL_GPIO_WritePin(in4_2_GPIO_Port, in4_2_Pin, SET);
			}
			else if (d == negative)
			{
				HAL_GPIO_WritePin(in4_1_GPIO_Port, in4_1_Pin, SET);
				HAL_GPIO_WritePin(in4_2_GPIO_Port, in4_2_Pin, RESET);
			}
			break;
  }
}

void MOTOR_Rotate(Turn d){
	float anglee = GetYaw();
	int pwm_rotate;
	const int max_pwm = 1300;
	if (anglee >= 0 && anglee < 90) pwm_rotate = abs(PID_Calculate_S(&pid_rotate, 0, anglee));
	else if (anglee <= 360 && anglee > 270)	pwm_rotate = abs(PID_Calculate_S(&pid_rotate, 360, anglee));
	if (anglee > 0 && anglee < 5) pwm_rotate = 1200;
	else if (anglee > 355 && anglee < 360) pwm_rotate = 1200;
	else if (anglee > 0 && anglee < 0.5) pwm_rotate = 800;
	else if (anglee > 357 && anglee < 360) pwm_rotate = 800;
	else if (anglee > 0 && anglee < 0.2) pwm_rotate = 400;
	else if (anglee > 359 && anglee < 360) pwm_rotate = 400;
	if(pwm_rotate > max_pwm) pwm_rotate = max_pwm;
	if (d == positive){
		MOTOR_Direction(negative, 1, pwm_rotate);
		MOTOR_Direction(positive, 2, pwm_rotate);
		MOTOR_Direction(positive, 3, pwm_rotate);
		MOTOR_Direction(negative, 4, pwm_rotate);
	}
	if (d == negative){
		MOTOR_Direction(negative, 3, pwm_rotate);
		MOTOR_Direction(positive, 4, pwm_rotate);
		MOTOR_Direction(positive, 1, pwm_rotate);
		MOTOR_Direction(negative, 2, pwm_rotate);
	}
}

void MOTOR_Straight(Direction d, int16_t pwm)
{
	if (d == forward)
	{
		MOTOR_Direction(positive, 1, pwm);
		MOTOR_Direction(positive, 3, pwm);
	}
	else if (d == back)
	{
		MOTOR_Direction(negative, 1, pwm);
		MOTOR_Direction(negative, 3, pwm);
	}
	else if (d == left)
	{
		MOTOR_Direction(positive, 2, pwm);
		MOTOR_Direction(positive, 4, pwm);
	}
	else if (d == right)
	{
		MOTOR_Direction(negative, 2, pwm);
		MOTOR_Direction(negative, 4, pwm);
	}
}


float MOTOR_CountSpeed(uint8_t index)
{
  int32_t cnt = 0;
  switch (index)
  {
  	  case 1:
  		  cnt = __HAL_TIM_GET_COUNTER(&htim2);
  		  cnt = (int16_t)cnt;
  		  __HAL_TIM_SET_COUNTER(&htim2, 0);
  		  return (float)cnt * 1000 / circle * 2 * PI * radius;
  		  break;
  	  case 2:
  		  cnt = __HAL_TIM_GET_COUNTER(&htim3);
  		  cnt = (int16_t)cnt;
  		  __HAL_TIM_SET_COUNTER(&htim3, 0);
  		  return (float)cnt * 1000 / circle * 2 * PI * radius;
  		  break;
  	  case 3:
  		  cnt = __HAL_TIM_GET_COUNTER(&htim5);
  		  cnt = (int16_t)cnt;
  		  __HAL_TIM_SET_COUNTER(&htim5, 0);
  		  return (float)cnt * 1000 / circle * 2 * PI * radius;
  		  break;
  	  case 4:
  		  cnt = __HAL_TIM_GET_COUNTER(&htim8);
  		  cnt = (int16_t)cnt;
  		  __HAL_TIM_SET_COUNTER(&htim8, 0);
  		  return (float)cnt * 1000 / circle * 2 * PI * radius;
  		  break;
  }
}

void MOTOR_Standby()
{
	for (int index = 1; index <= 4; index++)
		MOTOR_Direction(forward, index, 0);
}


void MOTOR_Move(Position_edc24 destination)
{
	now = getVehiclePos();
	int PWM_x = abs (destination.x - now.x) >= 8 ? PID_Calculate_S(&pid_x, (float)destination.x, (float)now.x) : 0;
	int PWM_y = abs (destination.y - now.y) >= 8 ? PID_Calculate_S(&pid_y, (float)destination.y, (float)now.y) : 0;
	u1_printf("PWM_x=%d ", PWM_x);
	u1_printf("%PWM_y=%d\n",PWM_y);
	if (PWM_x >= 0)	MOTOR_Straight(right, PWM_x);
	else if (PWM_x < 0)	MOTOR_Straight(left, -PWM_x);
	if (PWM_y >= 0)	MOTOR_Straight(forward, PWM_y);
	else if (PWM_y < 0)	MOTOR_Straight(back, -PWM_y);
}



