#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "FSM_task.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "chassiss_calc.h"
#include "INS_Task.h"
#include "gimbal_calc.h"
#include "CAN_receive.h"
extern osThreadId FSM_TASKHandle;
state_chassiss_list chassiss_state;
state_gimbal_list gimbal_state;

extern fp32 set_yaw;
extern fp32 INS_angle_deg[3];
extern fp32 set_v;
extern fp32 set_angle;
fp32 x;
fp32 y;
angle_f angle_secondmode;
angle_f angle_thirdmode;

void CHASSISS_PREPARE_SELF_CHECK(void);
void RC_chassiss_crol(void);
void GIMBAL_PREPARE_SELF_CHECK(void);
void RC_gimbal_crol(void);
void RC_GIMBAL_STAND(void);
void RC_GIMBAL_AUTO(void);

void chassiss_state_judge(state_chassiss_list state){
	switch(state){
		case CHASSISS_PREPARE:
				CHASSISS_PREPARE_SELF_CHECK();
		break;
		case CHASSISS_ENERGEGIVE_RC:
				CHASSISS_PREPARE_SELF_CHECK();
				RC_chassiss_crol();
		break;
		case CHASSISS_AUTOAIM:
			
		break;
		case CHASSISS_ENERGYLOSE:
			while(1){
				CAN_CMD_BASE(&hcan1,0x200,0,0,0,0);
				CAN_CMD_BASE(&hcan1,0x1FF,0,0,0,0);
			}
	}
}

void gimbal_state_judge(state_gimbal_list state,state_chassiss_list chassiss_state){
	switch(state){
		case GIMBAL_PREPARE:
				GIMBAL_PREPARE_SELF_CHECK();
		break;
		case GIMBAL_ENERGEGIVE_RC:
				GIMBAL_PREPARE_SELF_CHECK();
				RC_gimbal_crol();
		break;
		case GIMBAL_AUTOAIM:
				switch(chassiss_state){
					case CHASSISS_PREPARE:
							GIMBAL_PREPARE_SELF_CHECK();
							RC_chassiss_crol();
							RC_GIMBAL_STAND();
					break;
					case CHASSISS_ENERGEGIVE_RC:
							GIMBAL_PREPARE_SELF_CHECK();
							RC_gimbal_crol();
							RC_GIMBAL_AUTO();
							RC_GIMBAL_STAND();
					break;
				}
		break;
		case GIMBAL_SECONDMODE:
				GIMBAL_PREPARE_SELF_CHECK();
				RC_GIMBAL_STAND();
	}
}

void CHASSISS_PREPARE_SELF_CHECK(void){
	if(rc_ctrl.rc.s[1]==1){
			chassiss_state=0;
	}
	if(rc_ctrl.rc.s[1]==3){
			chassiss_state=1;
	}
	if(rc_ctrl.rc.s[1]==2){
			chassiss_state=3;
	}
}

void GIMBAL_PREPARE_SELF_CHECK(void){
	
	if(rc_ctrl.rc.s[0]==1){
			gimbal_state=2;
	}
	if(rc_ctrl.rc.s[0]==3){
			gimbal_state=1;
	}
	if(rc_ctrl.rc.s[0]==2){
			gimbal_state=3;
	}
}

void RC_GIMBAL_AUTO(void){
	angle_thirdmode.current_angle=(get_yaw_gimbal_motor_measure_point()->ecd/8191.0f )*360;	
	angle_thirdmode.last_angle=(get_yaw_gimbal_motor_measure_point()->last_ecd/8191.0f )*360;
	angle_thirdmode.change_angle=angle_thirdmode.current_angle-angle_thirdmode.last_angle;
	if(angle_thirdmode.change_angle>180){
		angle_thirdmode.change_angle-=360;
	}
	if(angle_thirdmode.change_angle<-180){
		angle_thirdmode.change_angle+=360;
	}
	if(set_yaw>360)set_yaw-=360;
	if(set_yaw<-360)set_yaw+=360;
	set_yaw+=angle_thirdmode.change_angle;
	RC_GIMBAL_STAND();
}



void RC_GIMBAL_STAND(void){
	angle_secondmode.current_angle=INS_angle_deg[0];
	angle_secondmode.change_angle=angle_secondmode.current_angle-angle_secondmode.last_angle;
	angle_secondmode.last_angle=INS_angle_deg[0];
		if(angle_secondmode.change_angle>180){
		angle_secondmode.change_angle-=360;
	}
	if(angle_secondmode.change_angle<-180){
		angle_secondmode.change_angle+=360;
	}
	if(set_angle>360)set_angle-=360;
	if(set_angle<-360)set_angle+=360;
	set_angle-=angle_secondmode.change_angle;
} 
	
void RC_chassiss_crol(void){
	if(rc_ctrl.rc.ch[1]<mid_rc&&rc_ctrl.rc.ch[1]>-1*mid_rc){
		set_v=0;
	}
	if(rc_ctrl.rc.ch[1]>mid_rc){
			set_v=30;
	}
	if(rc_ctrl.rc.ch[1]>min_rc&&rc_ctrl.rc.ch[1]<-1*mid_rc){
			set_v=-30;
	}
	if(rc_ctrl.rc.ch[0]>=mid_rc){
		if(set_yaw>359.85){
			set_yaw-=360;
		}
		set_v=0;		
		set_yaw+=0.15;
	}
	if(rc_ctrl.rc.ch[0]>=min_rc&&rc_ctrl.rc.ch[0]<=-1*mid_rc){
		if(set_yaw<0.15){
			set_yaw+=360;
		}
		set_v=0;		
		set_yaw-=0.15;
	}
}

void RC_gimbal_crol(void){
	if(rc_ctrl.rc.ch[2]>mid_rc){
	if(set_angle<0.15){
			set_angle+=360;
		}
		set_angle-=0.15;
	}
	if(rc_ctrl.rc.ch[2]>=min_rc&&rc_ctrl.rc.ch[2]<=-1*mid_rc){
		if(set_angle>359.85){
			set_angle-=360;
		}	
		set_angle+=0.15;
	}
	}

void FSM_begin(void){
	chassiss_state_judge(chassiss_state);
	gimbal_state_judge(gimbal_state,chassiss_state);
}
	
void FSM_task(void const * argument){
	remote_control_init();
	while(1){
		FSM_begin();
		x=rc_ctrl.rc.ch[0];
		y=rc_ctrl.rc.ch[1];
		vTaskDelay(1);
	}
}