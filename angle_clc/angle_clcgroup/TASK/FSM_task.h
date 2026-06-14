#ifndef FSM_TASK_H
#define FSM_TASK_H
#define mid_rc  200
#define max_rc  660
#define min_rc  -660
typedef enum {
	CHASSISS_PREPARE=0,
	CHASSISS_ENERGEGIVE_RC,
	CHASSISS_AUTOAIM,
	CHASSISS_ENERGYLOSE
}state_chassiss_list;

typedef enum {
	GIMBAL_PREPARE=0,
	GIMBAL_ENERGEGIVE_RC,
	GIMBAL_AUTOAIM,
	GIMBAL_SECONDMODE
}state_gimbal_list;

typedef struct {
	float last_angle;
	float current_angle;
	float change_angle;
}angle_f;

extern angle_f angle_secondmode;
extern state_gimbal_list gimbal_state;
extern state_chassiss_list chassiss_state;
#endif