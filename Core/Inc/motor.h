#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"
#include "tim.h"
#include "zigbee_edc24.h"
#include "algorithm.h"
#include  "pid.h"

#define circle 1060
#define radius 3
#define PI 3.14159

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
void MOTOR_Standby(void);
void MOTOR_Move(Position_edc24 destination);
void MOTOR_Rotate(Turn d);

extern MOTOR_Typedef motor[5];			//4个电机的方向和速度

extern int PWM[5];						//电机的PWM输入量
extern float motor_speed_x, motor_speed_y;
#endif

