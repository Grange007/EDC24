#ifndef __PID__
#define __PID__

#include "motor.h"
#include "usart.h"
typedef enum{
	straight_x,
	straight_y,
	rotate
}PID_Instance_S;

typedef struct{
	float Kp_ex;        //外环P
	float Kp, Ki, Kd;     //内环PID

	float Motor_speed;     //电机当前的速度
	float Position_now;     //小车当前的位置，由当前速度积分得出
	float target_speed;     //我们需要电机达到的速度,外环的输出，也就是内环的输入
	float target_position;      //我们需要小车达到的位置，由摇杆积分得出

	float err_position_now;     //当前小车位置的误差
	float err_position_last;    //上一次小车位置的误差

	float err_speed_now;      //当前速度与电机期望值的差，也就是当前的误差值
	float err_speed_last;     //上一次计算的误差值

	float err_speed_i;      //积分项，将累计时间内所有的误差值
	int output;       //经过PID算法后输出的数据，其实是一个控制PWM占空比的参数，我们只需要直接放在输出PWM的函数里就好了
	PID_Instance_S instance;
}PID_typedef_S;


void PID_Init_S(PID_typedef_S *pid, float Kp_ex_set, float Kp_set, float Ki_set, float Kd_set, PID_Instance_S instance_set);
int PID_Calculate_S(PID_typedef_S *pid, float set_value, float now_value);
void PID_Clear_S(PID_typedef_S *pid);

extern uint32_t pid_cnt;

extern PID_typedef_S pid_x, pid_y, pid_rotate;
extern float p_ex_set;
extern float p_set;						//设置pid的系数
extern float i_set;
extern float d_set;
extern float rp_ex_set;
extern float rp_set;
extern float ri_set;
extern float rd_set;
////外环PID
//void PID_OUT(){
//	err_position_now = target_position - position_now;
//	target_speed = Kp_ex *err_position_now;//外环的输出就是内环的输入
//	//速度的限幅处理略
//	err_position_last = err_position_now;
//}
//
//
////内环PID
//void PID_IN(){
//	err_speed_now = target_speed - Motor1_speed;
//	errspeed__i += err_speed_now;
//	if(err_speed_i > x) err_speed_i = x;//这里需要设定一个极限值，以防积分变量溢出，其实一般来说并不会达到这个极限值，因为误差并不都是正数
//	//没有对D进行操作
//	output = Kp * err_speed_now + Ki * err_speed_i +Kd * (err_speed_now - err_speed_last);//PID公式
//	if(output > x) output = x;//这里需要对输出也进行一个限制，这里倒不是限制溢出，而是PWM输出的参数本身就有最大值，超过了就无效了。
//	TIM_SetCompare1(TIMx, output);//对产生PWM的定时器通道一进行输出
//	err_speed_last = err_speed_now;//将使用完的当前误差值赋给上一次的误差值，进行下一轮循环
//}


#endif
