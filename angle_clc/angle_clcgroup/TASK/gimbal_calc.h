#ifndef GIMBAL_CALC__H
#define	GIMBAL_CALC__H
#include "pid.h"
typedef struct{
	uint16_t cnt;
	uint16_t last_cnt;
	float speed;
	float angle;
}data_t;

extern fp32 current_gimbal;
extern fp32 set_angle,set_speed;
#endif