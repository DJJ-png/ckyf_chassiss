#ifndef CHASSISS_CALC__H
#define CHASSISS_CALC__H
#include "pid.h"
    typedef struct{
            float speed_0;
            float speed_1;
            float speed_2;
            float speed_3;
    }speed_f;
    
        typedef struct{
            float current_0;
            float current_1;
            float current_2;
            float current_3;
    }current_f;

        typedef struct{
            float motor_0;
            float motor_1;
            float motor_2;
            float motor_3;
    }motor_f;
				
		extern current_f current;
		extern fp32 set_yaw;
		extern fp32 set_v;
#endif