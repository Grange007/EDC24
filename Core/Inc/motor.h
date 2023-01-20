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
	back
}Direction;

typedef struct
{
	Direction dic;
	int spd;
}MOTOR_Typedef;

void MOTOR_Direction(Direction d);
float MOTOR_CountSpeed();

extern MOTOR_Typedef motor;
extern PID_Typedef pid_speed;

extern float SPEED;
extern float SET_SPEED;
extern int PWM;
extern float p_set;
extern float i_set;
extern float d_set;

#endif

