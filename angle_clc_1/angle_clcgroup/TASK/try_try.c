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
void Try_Try(void const * argument){

	while(1){
			if(flag){
			CAN_CMD_BASE(&hcan1,0x200,current.current_0,current.current_1,-1*current.current_2,current.current_3);
			}
		  vTaskDelay(5);
			 
	}
}