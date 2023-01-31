#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"
#include "tim.h"

#define PID_MAX 1000
#define PID_MIN -1000

#define circle 1060
#define radius 3
#define PI 3.14159

typedef struct
{
	float Kp, Ki, Kd;
	float P, I, D;
	float Error_Last;
}PID_Typedef;

void PID_Init(PID_Typedef *pid, float p_set, float i_set, float d_set);
void PID_Clear(PID_Typedef *pid);
int PID_Calculate(PID_Typedef *pid, float set_value, float now_value );

typedef enum
{
	forward,
	back,
	left,
	right
}Direction;			//小车行驶方向

typedef enum
{
	positive,
	negative
}Turn;				//电机转动方向

typedef struct
{
	Turn dic;
	int spd;
}MOTOR_Typedef;

void MOTOR_Direction(Turn d, uint8_t index, int16_t pwm);
void MOTOR_Straight(Direction d, int16_t pwm);
float MOTOR_CountSpeed(uint8_t index);

extern MOTOR_Typedef motor[5];			//4个电机的方向和速度
extern PID_Typedef pid_speed[5];		//4个电机的pid

extern int PWM[5];						//电机的PWM输入量


extern float p_set;						//设置pid的系数
extern float i_set;
extern float d_set;

#endif

