#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "gimbal_calc.h"
#include "CAN_receive.h"
#include "chassiss_calc.h"
extern osThreadId GIMBAL_CALCHandle;

data_t data;

void read_cnt(data_t *data){
	
}
void gimbal_calc(void const * argument){
	while(1){
	vTaskDelay(1);
	}
}