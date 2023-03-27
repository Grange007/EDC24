#include "algorithm.h"
#include "motor.h"

Order_edc24 order[1005];
Position_edc24 transpoint[5] = {
		{0, 0},
		{127, 39},
		{39, 127},
		{127, 215},
		{215, 127}
};
Position_edc24 now;
Position_edc24 path[10];
SendStatus send_status;

int16_t num_of_order;//int16_t
int8_t cnt;//目前的道路中一共有的中转点的数量
int8_t cnt_run = 0;//找到自己在目前的道路中正在前往哪个中转点
Position_edc24 next_point;


Position_edc24 pos_pair(int x,int y)
{
//为了方便，把x,y转化成position_edc24类型的点
	Position_edc24 tmp;
	tmp.x=x;
	tmp.y=y;
	return tmp;
}

uint16_t dis(Position_edc24 a,Position_edc24 b)
{
//求曼哈顿距离
	return abs(a.x-b.x)+abs(a.y-b.y);
}

bool check_cross_wall(Position_edc24 a,Position_edc24 b)
{
//判断路线是否穿过墙
//0没穿过，1穿过
	if(a.x==b.x&&a.y==b.y)
		return 0;
	if(a.x==b.x)
	{
		if((a.x>=38&&a.x<=107)||(a.x>=147)&&(a.x<=216))
			if((a.y>=39&&b.y<=39)||(a.y<=39&&b.y>=39)||(a.y>=215&&b.y<=215)||(a.y<=215&&b.y>=215))
				return 1;
		return 0;
	}
	if(a.y==b.y)
	{
		if((a.y>=38&&a.y<=107)||(a.y>=147)&&(a.y<=216))
			if((a.x>=39&&b.x<=39)||(a.x<=39&&b.x>=39)||(a.x>=215&&b.x<=215)||(a.x<=215&&b.x>=215))
				return 1;
		return 0;
	}

	int8_t ex=6;
	if((a.y<=39&&b.y>=39)||(a.y>=39&&b.y<=39))
	{
		double x1=1.0*(39-b.y)*(a.x-b.x)/(a.y-b.y)+b.x;
		if((x1>=38-ex&&x1<=107+ex)||(x1>=147-ex&&x1<=216+ex))
			return 1;
	}
	if((a.y<=215&&b.y>=215)||(a.y>=215&&b.y<=215))
	{
		double x2=1.0*(215-b.y)*(a.x-b.x)/(a.y-b.y)+b.x;
		if((x2>=38-ex&&x2<=107+ex)||(x2>=147-ex&&x2<=216+ex))
			return 1;
	}
	if((a.x<=39&&b.x>=39)||(a.x>=39&&b.x<=39))
	{
		double y1=1.0*(39-b.x)*(a.y-b.y)/(a.x-b.x)+b.y;
		if((y1>=38-ex&&y1<=107+ex)||(y1>=147-ex&&y1<=216+ex))
			return 1;
	}
	if((a.x<=215&&b.x>=215)||(a.x>=215&&b.x<=215))
	{
		double y2=1.0*(215-b.x)*(a.y-b.y)/(a.x-b.x)+b.y;
		if((y2>=38-ex&&y2<=107+ex)||(y2>=147-ex&&y2<=216+ex))
			return 1;
	}

	return 0;
}

Position_edc24 get_nearest_transpoint(Position_edc24 a)
{
//找出最近的中转点
//中转点有四个，在门口中间
	uint16_t min_dis=1000;
	Position_edc24 tmp;
	for(int8_t i=1;i<=4;++i)
	{
		if(a.x==transpoint[i].x&&a.y==transpoint[i].y)
			continue;
		uint16_t d=dis(transpoint[i],a);
		if(min_dis>d)
		{
			min_dis=d;
			tmp=transpoint[i];
		}
	}
	return tmp;
}

Position_edc24 get_extension_transpoint(Position_edc24 a,Position_edc24 b)
{
	if(!check_cross_wall(pos_pair(a.x,b.y),a)&&!check_cross_wall(pos_pair(a.x,b.y),b))
		return pos_pair(a.x,b.y);
	else
		return pos_pair(b.x,a.y);
}
void extend_path(Position_edc24 a,Position_edc24 b)
{
//因为只能一个方向运动，所以加了这个函数
//将斜路线转化成水平和竖直的路线
//extension_transpoint就是那个改方向的点
	if(a.x!=b.x&&a.y!=b.y)
		path[++cnt]=get_extension_transpoint(a,b);
	path[++cnt]=b;
}

//void get_path(Position_edc24 destination)
//{
////找路
//	memset(path, 0, sizeof(path));
//	Position_edc24 nearest_transpoint;
//	now = getVehiclePos();
//	cnt=0;
//	path[cnt]=now;
//	if(check_cross_wall(path[cnt],destination))
//	{
//		nearest_transpoint=get_nearest_transpoint(now);
//		extend_path(path[cnt],nearest_transpoint);
//		if(check_cross_wall(path[cnt],destination))
//		{
//			nearest_transpoint=get_nearest_transpoint(destination);
//			extend_path(path[cnt],nearest_transpoint);
//		}
//	}
//	extend_path(path[cnt],destination);
//}


void get_path(Position_edc24 destination)
{
	 cnt=0;
	 path[cnt]=now;
	 if(check_cross_wall(now,destination))
	 {
		 if(check_cross_wall(now,get_nearest_transpoint(destination)))
			 extend_path(now,get_nearest_transpoint(now));
		 extend_path(path[cnt],get_nearest_transpoint(destination));
	 }
	 extend_path(path[cnt],destination);
}

void output_path()
{
//输出路线
//cnt是path里存的点数
//path里存的是每一个要经过的点
	printf("cnt:%d\n",cnt);
	for(int8_t i=1;i<=cnt;++i)
		printf("%d %d\n",path[i].x,path[i].y);
	memset(path, 0, sizeof(path));
}

Position_edc24 find_point()
{
	cnt_run = 0;
	for (int8_t i = 1; i <= cnt; i++)
	{
		int x1 = getVehiclePos().x, y1 = getVehiclePos().y;
		int x2 = path[i].x, y2 = path[i].y;
		if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) <= 64)
			if (getVehiclePos().x != 0 && getVehiclePos().y != 0)
				cnt_run = i;
	}
	return path[cnt_run + 1];
}

void orderInit()
{
		order_sending.depPos.x = 0;
		order_sending.depPos.y = 0;
		order_sending.desPos.x = 0;
		order_sending.desPos.y = 0;
		memset(path, 0, sizeof(path));
		cnt = 0;
		cnt_run = 0;
		PID_Clear_S(&pid_x);
		PID_Clear_S(&pid_y);

}

