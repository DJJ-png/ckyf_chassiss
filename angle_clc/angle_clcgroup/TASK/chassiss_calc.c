#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "chassiss_calc.h"
#include "math.h"
#include "CAN_receive.h"
#define error 0
extern osThreadId CHASSISS_CALCHandle;
speed_f speed;
pid_type_def motor_s,motor_p;
current_f current;
motor_f motor;
extern fp32 INS_angle_deg[3];
fp32 yaw=0;
fp32 set_yaw,yaw,yaw_cal;
fp32 kp_s=6.0f,ki_s=0.0f,kd_s=10.0f;
fp32 kp_p=6.0f,ki_p=0.0f,kd_p=10.0f;
fp32 set_omega;
fp32 set_v,vx,vy;
fp32 rx=0.14195,ry=0.109,s=0.0765;//单位为米
float omega_calc(fp32 previous_yaw,fp32 now_yaw,fp32 time){//time为ms
        fp32 omega;
        if(now_yaw>previous_yaw){
            omega=(now_yaw-previous_yaw)/time*1000.0f;
            return omega;
        }
        else{
            return error;
        }
}

float yaw_calc(float yaw,float set_yaw){
	fp32 yaw_after;
	yaw_after=yaw+180;
	yaw_after-=set_yaw;
	return yaw_after;
}

void PID_INIT(void){
        float PID1[3]={kp_s,ki_s,kd_s};
        float PID2[3]={kp_p,ki_p,kd_p};
				PID_init(&motor_s, PID_POSITION, PID1,2000,150);//PID初始化
				PID_init(&motor_p, PID_POSITION, PID2,2000,80);//PID初始化
}


float angle_calc(pid_type_def *pid_p,fp32 set_angle,fp32 angle){
		fp32 set_omega;
    PID_calc(pid_p,angle,set_angle);
    set_omega=pid_p->out;
    return set_omega;
}
float vx_calc(fp32 set_v,fp32 yaw){
    vx=set_v*cosf(set_yaw);
		return vx;
}

float vy_calc(fp32 set_v,fp32 yaw){
    vy=set_v*sinf(set_yaw);
		return vy;
}

void pid_calc(motor_f*motor,fp32 vx,fp32 vy,fp32 omega){
    fp32 v_x,v_y,wheel_v_x,wheel_v_y;
		wheel_v_x=omega*rx;
		wheel_v_y=-1*omega*ry;
	
		motor->motor_0=(-1*vx+vy+omega*(rx+ry))/s;
		motor->motor_1=(-1*vx+-1*vy+omega*(rx+ry))/s;
		motor->motor_2=(vx-vy+omega*(rx+ry))/s;
		motor->motor_3=(vx+vy+omega*(rx+ry))/s;
		
}

void chassiss_prevent_skid(motor_f *motor,fp32 now_omega,fp32 set_omega,pid_type_def *motors,current_f *current){
	fp32 motor_correct;
	PID_calc(motors,now_omega,set_omega);
	motor_correct=motors->out;
	motor->motor_0+=motor_correct;
	motor->motor_1+=motor_correct;
	motor->motor_2+=motor_correct;
	motor->motor_3+=motor_correct;
	
}

void current_calc(pid_type_def *pid,current_f *current,motor_f *motor,speed_f *speed){
	PID_calc(&motor_s,speed->speed_0,motor->motor_0);
	current->current_0=motor_s.out;
	PID_calc(&motor_s,speed->speed_1,motor->motor_1);
	current->current_1=motor_s.out;
	PID_calc(&motor_s,speed->speed_2,motor->motor_2);
	current->current_2=motor_s.out;
	PID_calc(&motor_s,speed->speed_3,motor->motor_3);
	current->current_3=motor_s.out;
}

void rpm_receive(speed_f *speed){
	speed->speed_0=get_chassis_motor_measure_point(0)->speed_rpm/36.0f;
	speed->speed_1=get_chassis_motor_measure_point(1)->speed_rpm/36.0f;
	speed->speed_2=get_chassis_motor_measure_point(2)->speed_rpm/36.0f;
	speed->speed_3=get_chassis_motor_measure_point(3)->speed_rpm/36.0f;
}

void PID_CROL(void){
	set_omega=0.2*angle_calc(&motor_p,yaw_cal,set_yaw);  
	vx=vx_calc(set_v,INS_angle_deg[0]);
	vy=vy_calc(set_v,INS_angle_deg[0]);
	pid_calc(&motor,vx,vy,set_omega);
	current_calc(&motor_s,&current,&motor,&speed);
}
void chassiss_calc(void const * argument){
        PID_INIT();
        while(1){
					yaw=INS_angle_deg[0];
					rpm_receive(&speed);
					yaw_cal=yaw_calc(yaw,set_yaw);
					set_yaw=70;
					PID_CROL();
					vTaskDelay(1);
        }
}