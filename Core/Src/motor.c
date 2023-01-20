#include "motor.h"

MOTOR_Typedef motor;
PID_Typedef pid_speed;

float SPEED;
float SET_SPEED = 160;
int PWM;
float p_set = 4.f;
float i_set = 0.1f;
float d_set = 0.f;

void MOTOR_Direction(Direction d)
{
  if (d == forward)
  {
    motor.dic = forward;
    HAL_GPIO_WritePin(in1_1_GPIO_Port, in1_1_Pin, SET);
    HAL_GPIO_WritePin(in1_2_GPIO_Port, in1_2_Pin, RESET);
    HAL_GPIO_WritePin(in2_3_GPIO_Port, in2_3_Pin, SET);
	HAL_GPIO_WritePin(in2_4_GPIO_Port, in2_4_Pin, RESET);
  }
  else
  {
    motor.dic = back;
    HAL_GPIO_WritePin(in1_1_GPIO_Port, in1_1_Pin, RESET);
    HAL_GPIO_WritePin(in1_2_GPIO_Port, in1_2_Pin, SET);
    HAL_GPIO_WritePin(in2_3_GPIO_Port, in2_3_Pin, RESET);
	HAL_GPIO_WritePin(in2_4_GPIO_Port, in2_4_Pin, SET);
  }
}

float MOTOR_CountSpeed()
{
  int32_t cnt = 0;
  cnt = __HAL_TIM_GET_COUNTER(&htim2);
  cnt = (int16_t)cnt;
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  return (float)cnt * 1000 / circle * 2 * PI * radius;
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
