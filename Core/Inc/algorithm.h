#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "main.h"
#include "zigbee_edc24.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define orderMax 5

typedef enum
{
	unknown=0,
	waiting,
	loading,
	finishing
}OrderStatus;

extern Position_edc24 transpoint[5];
extern Position_edc24 now;
extern Position_edc24 path[10];

extern int8_t cnt;
extern Position_edc24 next_point;

extern uint16_t order_cnt;
extern int16_t order_id;
extern Order_edc24 order[1005];
extern OrderStatus order_status[1005];

extern bool charge;
extern bool pile[3];


Position_edc24 pos_pair(int16_t x,int16_t y);
uint16_t dis(Position_edc24 a,Position_edc24 b);
bool check_cross_wall(Position_edc24 a,Position_edc24 b);
Position_edc24 get_nearest_transpoint(Position_edc24 a);
Position_edc24 get_extension_transpoint(Position_edc24 a,Position_edc24 b);
void extend_path(Position_edc24 a,Position_edc24 b);
void get_path(Position_edc24 destination);
void output_path();

Position_edc24 get_nearest_point();
void store_order();
Position_edc24 check_power();
void set_pile();

Position_edc24 find_point();
void orderInit(void);


#endif /* INC_ALGORITHM_H_ */
