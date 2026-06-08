#ifndef FSM_TASK_H
#define FSM_TASK_H
#define mid_rc  700
#define max_rc  1664
#define min_rc  200
typedef enum {
	CHASSISS_PREPARE=0,
	CHASSISS_ENERGEGIVE_RC,
	CHASSISS_AUTOAIM,
	CHASSISS_ENERGYLOSE
}state_list;
#endif