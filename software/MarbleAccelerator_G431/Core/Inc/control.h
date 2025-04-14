#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

//#define SINGLECOIL_DESIGN
//#define CURRENT_B_WORKAROUND

#include <stdint.h>
#include <stdbool.h>

#include "ctrl_math.h"

typedef struct
{
	interp_lin_i16_t ia;
	interp_lin_i16_t ib;
	interp_lin_i16_t ic;
	bool i_autoOffs;
	interp_lin_i16_t vbus;
	interp_lin_i16_t temp;
	interp_lin_i16_t poti;
	uint16_t imax;
	uint16_t umax;
	uint16_t umin;
} calib_data_t;


typedef enum {
	ctrl_sm_state_startup = 0,
	ctrl_sm_state_curroffs = 1,
	ctrl_sm_state_ready = 3,
	ctrl_sm_state_vcon = 4,
	ctrl_sm_state_ccon = 5,
	ctrl_sm_state_off = 6,
	ctrl_sm_state_error = 8
} ctrl_sm_states_t;


typedef enum {
	ctrl_sm_cmd_none = 0,
	ctrl_sm_cmd_start = 1,
	ctrl_sm_cmd_stop = 2
} ctrl_sm_cmd_t;

typedef enum {
	ctrl_errors_none = 0,
	ctrl_errors_overcurrent = 1,
	ctrl_errors_overvoltage = 2,
	ctrl_errors_undervoltage = 3,
	ctrl_errors_adc = 4,
	ctrl_errors_currentmeas = 5,
	ctrl_errors_other = 6
} ctrl_errors_t;

typedef struct{
	uint8_t ticks;
	uint8_t ticks_per_ms;
	uint16_t ms;
	uint32_t s;
}
clock_data_t;

//Control data struct
typedef struct{
	int16_t ia;       //int16, Q7.8
	int16_t ib;       //int16, Q7.8
	int16_t ic;       //int16, Q7.8
	int16_t ua;       //int16, Q7.8, can be -vbus/2 ... vbus/2
	int16_t ub;       //int16, Q7.8, can be -vbus/2 ... vbus/2
	int16_t uc;       //int16, Q7.8, can be -vbus/2 ... vbus/2
	int16_t vbus;     //int16, Q7.8
	int16_t divVbus;  //int16, Q0.15
	int16_t pwmrefa;  //int16, Q0.15
	int16_t pwmrefb;  //int16, Q0.15
	int16_t pwmrefc;  //int16, Q0.15
	int16_t iaref;    //int16, Q7.8
	int16_t ibref;    //int16, Q7.8
	int16_t poti;
	control_pictrl_i16_t pi_a;
	control_pictrl_i16_t pi_b;
	bool button_pressed;  //indicate a button press
	bool trigger_in;      //indicate input (currently unused)
	uint32_t ctrl_sm_cnt;
	ctrl_sm_states_t state;
	ctrl_sm_cmd_t cmd;
	ctrl_errors_t error;
	int32_t ia_offs_sum;
	int32_t ib_offs_sum;
	int32_t ic_offs_sum;
	uint32_t timCnt_ctrl_begin;  //counter value at beginning of control routine
	uint32_t timCnt_ctrl_check;  //counter value at checkpoint in control routine
	uint32_t timCnt_ctrl_end;    //counter value at end of control routine
	clock_data_t clock;
} ctrl_data_t;


extern ctrl_data_t myctrl;
extern FDCAN_HandleTypeDef hfdcan1;

#endif //__CONTROL_H
