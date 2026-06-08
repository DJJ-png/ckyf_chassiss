#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "FSM_task.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "chassiss_calc.h"
#include "INS_Task.h"
extern osThreadId FSM_TASKHandle;
state_list state;

extern fp32 set_yaw;
extern fp32 INS_angle_deg[3];
extern fp32 set_v;
fp32 x;
void PREPARE_SELF_CHECK(void);
void RC_chassiss_crol(void);

void state_judge(state_list state){
	switch(state){
		case CHASSISS_PREPARE:
				PREPARE_SELF_CHECK();
		break;
		case CHASSISS_ENERGEGIVE_RC:
				RC_chassiss_crol();
		break;
		case CHASSISS_AUTOAIM:
			
		break;
		case CHASSISS_ENERGYLOSE:
			while(1){
				CAN_CMD_BASE(&hcan2,0x200,0,0,0,0);
			}
	}
}

void RC_chassiss_crol(void){
	if(rc_ctrl.rc.ch[0]>mid_rc){
		do{
			set_yaw=INS_angle_deg[0];
		}while(0);
			set_v=5;
	}
	if(rc_ctrl.rc.ch[0]>min_rc&&rc_ctrl.rc.ch[0]<mid_rc){
		do{
			set_yaw=INS_angle_deg[0];
		}while(0);
			set_v=-5;
	}
	if(rc_ctrl.rc.ch[1]>mid_rc){
		if(set_yaw>350){
			set_yaw-=360;
		}
		set_v=0;		
		set_yaw+=10;
	}
	if(rc_ctrl.rc.ch[1]>min_rc&&rc_ctrl.rc.ch[1]<mid_rc){
		if(set_yaw<10){
			set_yaw+=360;
		}
		set_v=0;		
		set_yaw-=10;
	}
}

void PREPARE_SELF_CHECK(void){
	
	if(rc_ctrl.rc.s[1]==3){
			state=1;
	}
}

void FSM_task(void const * argument){
	remote_control_init();
	while(1){
		x=rc_ctrl.rc.ch[0];
		vTaskDelay(1);
	}
}