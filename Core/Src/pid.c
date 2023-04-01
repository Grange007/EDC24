#include "pid.h"

PID_typedef_S pid_x, pid_y, pid_rotate;

float p_ex_set = 10.f;
float p_set = 4.f;
float i_set = 0.1f;
float d_set = 1.f;

float rp_ex_set = 15.f;
float rp_set = 3.f;
float ri_set = 0.25f;
float rd_set = 0.05f;
uint32_t pid_cnt;

void PID_Init_S(PID_typedef_S *pid, float Kp_ex_set, float Kp_set, float Ki_set, float Kd_set, PID_Instance_S instance_set){
	pid->Kp_ex = Kp_ex_set;
	pid->Kp = Kp_set;
	pid->Ki = Ki_set;
	pid->Kd = Kd_set;
	pid->Motor_speed = 0.f;     //电机当前的速度
	pid->Position_now = 0.f;     //小车当前的位置，由当前速度积分得出
	pid->target_speed = 0.f;     //我们需要电机达到的速度,外环的输出，也就是内环的输入
	pid->target_position = 0.f;      //我们需要小车达到的位置，由摇杆积分得出

	pid->err_position_now = 0.f;     //当前小车位置的误差
	pid->err_position_last = 0.f;    //上一次小车位置的误差

	pid->err_speed_now = 0.f;      //当前速度与电机期望值的差，也就是当前的误差值
	pid->err_speed_last = 0.f;     //上一次计算的误差值

	pid->err_speed_i = 0.f;      //积分项，将累计时间内所有的误差值
	pid->instance = instance_set;
}

void PID_Clear_S(PID_typedef_S *pid){
	pid->Motor_speed = 0.f;     //电机当前的速度
	pid->Position_now = 0.f;     //小车当前的位置，由当前速度积分得出
	pid->target_speed = 0.f;     //我们需要电机达到的速度,外环的输出，也就是内环的输入
	pid->target_position = 0.f;      //我们需要小车达到的位置，由摇杆积分得出

	pid->err_position_now = 0.f;     //当前小车位置的误差
	pid->err_position_last = 0.f;    //上一次小车位置的误差

	pid->err_speed_now = 0.f;      //当前速度与电机期望值的差，也就是当前的误差值
	pid->err_speed_last = 0.f;     //上一次计算的误差值

	pid->err_speed_i = 0.f;      //积分项，将累计时间内所有的误差值
}

int PID_Calculate_S(PID_typedef_S *pid, float set_value, float now_value){
	const int max_value = 8000;
	const int min_value = 3000;
	const int max_speed = 2000;
	if ((pid_cnt ++ % 2) == 0){
		pid->target_position = set_value;
		pid->Position_now = now_value;
		pid->err_position_now = pid->target_position - pid->Position_now;
		pid->target_speed = pid->Kp_ex * pid->err_position_now;//外环的输出就是内环的输入
		//速度的限幅处理略
		pid->err_position_last = pid->err_position_now;
		if (pid->target_speed > max_speed) pid->target_speed = max_speed;
		if (pid->target_speed < -max_speed) pid->target_speed = -max_speed;
	}
	if (pid->instance == straight_x) pid->Motor_speed = motor_speed_x;
	if (pid->instance == straight_y) pid->Motor_speed = motor_speed_y;
	if (pid->instance == rotate) pid->Motor_speed = motor_speed_x;
	pid->err_speed_now = pid->target_speed - pid->Motor_speed;
	pid->err_speed_i += pid->err_speed_now;
	if(pid->err_speed_i < -50000) pid->err_speed_i = -50000;//这里需要设定一个极限值，以防积分变量溢出，其实一般来说并不会达到这个极限值，因为误差并不都是正数
	if(pid->err_speed_i > 50000) pid->err_speed_i = 50000;//这里需要设定一个极限值，以防积分变量溢出，其实一般来说并不会达到这个极限值，因为误差并不都是正数
	pid->output = (int) pid->Kp * pid->err_speed_now + pid->Ki * pid->err_speed_i + pid->Kd * (pid->err_speed_now - pid->err_speed_last);//PID公式
	u1_printf("target: %f, x: %f, y: %f, out: %d\n", pid->target_speed, motor_speed_x, motor_speed_y, pid->output);
	pid->err_speed_last = pid->err_speed_now;//将使用完的当前误差值赋给上一次的误差值，进行下一轮循环
	if (pid->output > max_value) return max_value;
	if (pid->output < -max_value) return -max_value;
	if (pid->instance == straight_x || pid->instance == straight_y){
		if (pid->output < min_value && pid->output > 0) return min_value;
		if (pid->output > -min_value && pid->output < 0) return -min_value;
	}

	return pid->output;
}
