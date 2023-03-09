#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "main.h"
#include "zigbee_edc24.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef enum
{
	fetch,
	send
}SendStatus;

extern SendStatus send_status;
extern Order_edc24 order[1005];
extern Position_edc24 transpoint[5];
extern Position_edc24 now;
extern Position_edc24 path[10];

extern int16_t num_of_order;//int16_t
extern int8_t cnt;
extern int8_t cnt_run;//找到自己在目前的道路中正在前往哪个中转点
extern Position_edc24 next_point;


Position_edc24 pos_pair(int x,int y);
uint16_t dis(Position_edc24 a,Position_edc24 b);
bool check_cross_wall(Position_edc24 a,Position_edc24 b);
Position_edc24 get_nearest_transpoint(Position_edc24 a);
Position_edc24 get_extension_transpoint(Position_edc24 a,Position_edc24 b);
void extend_path(Position_edc24 a,Position_edc24 b);
void get_path(Position_edc24 destination);
void output_path();
Position_edc24 find_point();
void orderInit(void);


#endif /* INC_ALGORITHM_H_ */
