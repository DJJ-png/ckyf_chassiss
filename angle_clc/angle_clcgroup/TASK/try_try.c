#include "main.h"
#include "INS_Task.h"
#include "try_try.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "bsp_can.h"
#include "chassiss_calc.h"
extern osThreadId try_tryHandle;
uint8_t flag;
extern current_f current;
extern fp32 current_gimbal;
void Try_Try(void const * argument){

	while(1){
			if(1){
			CAN_CMD_BASE(&hcan1,0x200,current.current_0,current.current_1,current.current_2,current.current_3);
			CAN_CMD_BASE(&hcan1,0x1FF,current_gimbal,0,0,0);
			}
		  vTaskDelay(5);
			 
	}
}