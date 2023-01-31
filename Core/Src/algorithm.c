#include "algorithm.h"

Order_edc24 order[1005];
Position_edc24 transpoint[5];
Position_edc24 now;
Position_edc24 path[10];
SendStatus send_status;

int16_t num_of_order;//int16_t
int8_t cnt;

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

void get_path(Position_edc24 destination)
{
//找路
	Position_edc24 nearest_transpoint;
	now = getVehiclePos();
	cnt=0;
	path[cnt]=now;
	if(check_cross_wall(path[cnt],destination))
	{
		nearest_transpoint=get_nearest_transpoint(now);
		extend_path(path[cnt],nearest_transpoint);
		if(check_cross_wall(path[cnt],destination))
		{
			nearest_transpoint=get_nearest_transpoint(destination);
			extend_path(path[cnt],nearest_transpoint);
		}
	}
	extend_path(path[cnt],destination);
	now=destination;
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

//int main()
//{
//	//门口中间的中转点坐标
//	transpoint[1].x=127;
//	transpoint[1].y=39;
//	transpoint[2].x=39;
//	transpoint[2].y=127;
//	transpoint[3].x=127;
//	transpoint[3].y=215;
//	transpoint[4].x=215;
//	transpoint[4].y=127;
//
//	scanf("%d%d",&now .x,&now.y);//小车当前位置
//	scanf("%d",&num_of_order);//订单数量，可以没有，没有就把下面的for改成while
//
//	for(int8_t i=1;i<=num_of_order;++i)
//	{
//		scanf("%d%d",&provider[i].p.x,&provider[i].p.y);//订单信息
//		scanf("%d%d",&receiver[i].p.x,&receiver[i].p.y);
//
//		get_path(provider[i].p);//取外卖
//		output_path();
//		get_path(receiver[i].p);//送外卖
//		output_path();
//	}
//	return 0;
//}
