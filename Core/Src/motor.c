#include "motor.h"

MOTOR_Typedef motor[5];
PID_Typedef pid_speed[5];
PID_Typedef pid_x, pid_y;

int PWM[5];
float p_set = 4.f;
float i_set = 0.1f;
float d_set = 0.f;

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
				HAL_GPIO_WritePin(in2_1_GPIO_Port, in2_1_Pin, SET);
				HAL_GPIO_WritePin(in2_2_GPIO_Port, in2_2_Pin, RESET);

			}
			else if (d == negative)
			{
				HAL_GPIO_WritePin(in2_1_GPIO_Port, in2_1_Pin, RESET);
				HAL_GPIO_WritePin(in2_2_GPIO_Port, in2_2_Pin, SET);
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
				HAL_GPIO_WritePin(in4_1_GPIO_Port, in4_1_Pin, SET);
				HAL_GPIO_WritePin(in4_2_GPIO_Port, in4_2_Pin, RESET);
			}
			else if (d == negative)
			{
				HAL_GPIO_WritePin(in4_1_GPIO_Port, in4_1_Pin, RESET);
				HAL_GPIO_WritePin(in4_2_GPIO_Port, in4_2_Pin, SET);
			}
			break;
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




/**
  * @brief Initialize PID
  * @param pid: the pid controller
  * @param p_set: Kp of a specific pid controller
  * @param i_set: Ki of a specific pid controller
  * @param d_set: Kd of a specific pid controller
*/
void PID_Init(PID_Typedef *pid, float p_set, float i_set, float d_set)
{
  pid->Kp = p_set;
  pid->Ki = i_set;
  pid->Kd = d_set;
  pid->P = 0.f;
  pid->I = 0.f;
  pid->D = 0.f;
  pid->Error_Last = 0.f;
}

/**
 * @brief clear the former status of a pid controller
 * @param pid: the pid controller
*/
void PID_Clear(PID_Typedef *pid)
{
  pid->P = 0;
  pid->I = 0;
  pid->D = 0;
  pid->Error_Last = 0;
}


/**
 * @brief Calculate PID
 * @param pid: which pid controller
 * @param set_value: the target value
 * @param now_value: the current value
 * @return the pwm to be applied
*/
int PID_Calculate(PID_Typedef *pid, float set_value, float now_value )
{
	pid->P = set_value - now_value;
	pid->I += pid->P;
	pid->D = pid->P - pid->Error_Last;
	pid->Error_Last = pid->P;
//	pid->I = pid->I>10000 ? 10000 : (pid->I<(-10000) ? (-10000) : pid->I);
	if( set_value == 0 )			pid->I = 0;
	return(int)(pid->Kp*pid->P  +  pid->Ki*pid->I  +  pid->Kd*pid->D);
}


void MOTOR_Move(Position_edc24 destination)
{
	now = getVehiclePos();
	int PWM_x = PID_Calculate(&pid_x, (float)destination.x, (float)now.x);
	int PWM_y = PID_Calculate(&pid_y, (float_t)destination.y, (float)now.y);
	if (PWM_x > 0)	MOTOR_Straight(right, PWM_x);
	else if (PWM_x < 0)	MOTOR_Straight(left, -PWM_x);
	if (PWM_y > 0)	MOTOR_Straight(forward, PWM_y);
	else if (PWM_y < 0)	MOTOR_Straight(back, -PWM_y);
}

