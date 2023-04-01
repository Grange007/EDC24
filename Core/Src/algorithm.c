#include "algorithm.h"
#include "motor.h"

Position_edc24 transpoint[5] = {
		{0, 0},
		{127, 39},
		{39, 127},
		{127, 215},
		{215, 127}
};
Position_edc24 now;
Position_edc24 path[10];

int8_t cnt;//目前的道路中一共有的中转点的数量
Position_edc24 next_point;

uint16_t order_cnt;//总共生成的订单数量
int16_t order_id;
Order_edc24 order[1005];//所有订单信息
OrderStatus order_status[1005];//订单状态

bool charge;

Position_edc24 pos_pair(int16_t x,int16_t y)
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
		if((a.x>=38&&a.x<=107)||(a.x>=147&&a.x<=216))
			if((a.y>=39&&b.y<=39)||(a.y<=39&&b.y>=39)||(a.y>=215&&b.y<=215)||(a.y<=215&&b.y>=215))
				return 1;
		return 0;
	}
	if(a.y==b.y)
	{
		if((a.y>=38&&a.y<=107)||(a.y>=147&&a.y<=216))
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
	cnt=0;
	path[cnt]=now;
	if(check_cross_wall(now,destination))
	{
		if(check_cross_wall(now,get_nearest_transpoint(destination)))
			extend_path(now,get_nearest_transpoint(now));
		extend_path(path[cnt],get_nearest_transpoint(destination));
	}
	extend_path(path[cnt],destination);

	uint16_t sum_dis=0;
	for(int i=1;i<=cnt;++i)
		sum_dis+=dis(path[i],path[i-1]);
	if(sum_dis>2*dis(now,destination))
	{
		cnt=0;
		path[cnt]=now;
		extend_path(now,destination);
	}
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

Position_edc24 get_nearest_point()
{
	uint16_t min_dis=1000;
	uint16_t d;
	Position_edc24 tmp_pos=pos_pair(0,0);
	if(getOrderNum()<orderMax)
		for(uint16_t i=1;i<=order_cnt;++i)
		{
			d=dis(now,order[i].depPos);
			if(order_status[order[i].orderId]==waiting&&d<min_dis)
			{
				min_dis=d;
				tmp_pos=order[i].depPos;
				order_id=order[i].orderId;
			}
		}
	for(uint16_t i=1;i<=order_cnt;++i)
	{
		d=dis(now,order[i].desPos);
		if(order_status[order[i].orderId]==loading&&d<min_dis)
		{
			min_dis=d;
			tmp_pos=order[i].desPos;
			order_id=order[i].orderId;
		}
	}
	return tmp_pos;
}

void store_order()
{
	Order_edc24 tmp_order=getLatestPendingOrder();
	if(tmp_order.orderId!=0&&(order_status[tmp_order.orderId]==unknown||order_status[tmp_order.orderId]==finishing))
	{
		order_status[tmp_order.orderId]=waiting;
		order[++order_cnt]=tmp_order;
	}
//	u1_printf("\torder_cnt:%d orderId:%d\n",order_cnt,tmp_order.orderId);
}

Position_edc24 check_power()
{
	uint16_t min_dis=1000;
	uint8_t ex_dis=20;
	int32_t remain_dis=getRemainDist();
	Position_edc24 tmp;
	for(int i=1;i<=3;++i)
	{
		Position_edc24 tmp_pile=getOneOwnPile(i);
		if(min_dis>dis(now,tmp_pile))
		{
			min_dis=dis(now,tmp_pile);
			tmp=tmp_pile;
		}
	}
	if(min_dis+ex_dis>remain_dis)
		return tmp;
	return pos_pair(0,0);
}

void set_pile()
{
	now=getVehiclePos();
	if((now.x-39)*(now.x-39)+(now.y-127)*(now.y-127)<=64&&getOwnChargingPileNum()==0)
		setChargingPile();
	if((now.x-127)*(now.x-127)+(now.y-127)*(now.y-127)<=64&&getOwnChargingPileNum()==1)
		setChargingPile();
	if((now.x-215)*(now.x-215)+(now.y-127)*(now.y-127)<=64&&getOwnChargingPileNum()==2)
		setChargingPile();
	if(getOwnChargingPileNum()==0)
		next_point=pos_pair(39,127);
	if(getOwnChargingPileNum()==1)
		next_point=pos_pair(127,127);
	if(getOwnChargingPileNum()==2)
		next_point=pos_pair(215,127);
}

void orderInit()
{
		order_sending.depPos.x = 0;
		order_sending.depPos.y = 0;
		order_sending.desPos.x = 0;
		order_sending.desPos.y = 0;
		memset(path, 0, sizeof(path));
		memset(order, 0, sizeof(order));
		memset(order_status, 0, sizeof(order_status));
		cnt = 0;
		next_point=pos_pair(0,0);
		order_cnt = 0;
		charge=false;
		PID_Clear_S(&pid_x);
		PID_Clear_S(&pid_y);
}
