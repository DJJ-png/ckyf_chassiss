#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "gimbal_calc.h"
#include "CAN_receive.h"
#include "chassiss_calc.h"
#include "FSM_task.h"
#include "pid.h"
#define angle_relat 0
extern osThreadId GIMBAL_CALCHandle;

fp32 set_angle=285,set_speed;
fp32 kp_6020_s=6.0f,ki_6020_s=0.0f,kd_6020_s=0.0f;
fp32 kp_6020_p=6.0f,ki_6020_p=0.0f,kd_6020_p=0.0f;
fp32 set_angle_calc;
pid_type_def motor_6020_s,motor_6020_p;
data_t data;
fp32 current_gimbal;

void PID_6020_INIT(void){
        float PID1[3]={kp_6020_s,ki_6020_s,kd_6020_s};
        float PID2[3]={kp_6020_p,ki_6020_p,kd_6020_p};
				PID_init(&motor_6020_s, PID_POSITION, PID1,15000,150);//PID初始化
				PID_init(&motor_6020_p, PID_POSITION, PID2,15000,80);//PID初始化
}

void read_data(data_t *data){
	data->cnt=get_yaw_gimbal_motor_measure_point()->ecd;
	data->last_cnt=get_yaw_gimbal_motor_measure_point()->last_ecd;
	data->speed=get_yaw_gimbal_motor_measure_point()->speed_rpm;
}

void current_angle_calc(data_t*data){
	data->angle=(data->cnt/8191.0f)*360;
}

float setangle_calc(float set_angle,float angle){
	float error=set_angle-angle;
	if(error>180){
		error-=360;
	}
	if(error<-180){
		error+=360;
	}
	return error;
}

float PID_6020_CALC(pid_type_def *motor_p,pid_type_def *motor_s,data_t*data,fp32 set_angle,fp32 current_angle,fp32 current_gimbal){
	PID_calc(motor_p,current_angle,set_angle);
	set_speed=motor_p->out;
	PID_calc(motor_s,data->speed,set_speed);
	current_gimbal=motor_s->out;
	return current_gimbal;
}

//float current_change(float current){
//	if(current<=1000&&current>=500||current>=-1000&&current<=-500){
//		current*=4;
//	}
//	if(current<=500&&current>=100||current>=-500&&current<=-100){
//		current*=10;
//	}
//		if(current<=100&&current>=0||current>=-100&&current<=0){
//		current*=20;
//	}
//	return current;
//} 

void gimbal_begim(void){
	PID_6020_INIT();
	read_data(&data);
	current_angle_calc(&data);
	set_angle_calc=setangle_calc(set_angle,data.angle);
	current_gimbal=PID_6020_CALC(&motor_6020_p,&motor_6020_s,&data,set_angle_calc,angle_relat,current_gimbal);
//	current_change(current_gimbal);
}

void gimbal_calc(void const * argument){
	while(1){
	gimbal_begim();
	vTaskDelay(1);
	}
}