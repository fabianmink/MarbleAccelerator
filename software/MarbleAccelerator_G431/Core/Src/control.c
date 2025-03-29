/*
  Copyright (c) 2025 Fabian Mink <fabian.mink@gmx.de>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "control.h"
#define PWM_MAX 2500

//Power stage
static calib_data_t calib_data = {
		.ia = {.offs = 0, .gain = -1900},
		.ib = {.offs = 0, .gain = -1900},
		.ic = {.offs = 0, .gain = -1900},
		.i_autoOffs = true,
		.vbus = {.offs = 47, .gain = 551},
		.poti = {.offs = 0,  .gain = 256},
		.imax = 20 *256,   //20A
		.umin = 8  *256,   //8V
		.umax = 25 *256    //25V
};


void datarec_sm(void);
//void can_init(void);
//void can_sm(void);


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
	ctrl_errors_other = 5
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
	int16_t ua;       //int16, Q7.8
	int16_t ub;       //int16, Q7.8
	int16_t uc;       //int16, Q7.8
	int16_t vbus;     //int16, Q7.8
	int16_t divVbus;  //int16, Q0.15
	int16_t pwmrefa;  //int16, Q0.15
	int16_t pwmrefb;  //int16, Q0.15
	int16_t pwmrefc;  //int16, Q0.15
	int16_t iaref;    //int16, Q7.8
	int16_t ibref;    //int16, Q7.8
	int16_t iaval;    //int16, Q7.8
	int16_t ibval;    //int16, Q7.8
	uint32_t cnt_starta;
	uint32_t cnt_stopa;
	uint32_t cnt_startb;
	uint32_t cnt_stopb;
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

static ctrl_data_t myctrl;

typedef enum {
	sens_state_wft = 0,
	sens_state_run = 1, //inside sensor windows
	sens_state_fly = 2, //behind sensor windows, "free flying"
	sens_state_rdy = 3  //in coil
} sensor_states_t;

typedef enum {
	sens_cmd_none = 0,
	sens_cmd_reset = 1,
	sens_cmd_trigger = 2 //for manual triggering
}
sensor_cmd_t;

typedef struct{
	uint32_t cnt;
	uint16_t spd;
	uint32_t pos;
	uint32_t trigpos;
	uint16_t level;
	uint16_t hyst;
	sensor_states_t state;
	sensor_cmd_t cmd;
} sensor_data_t;

static sensor_data_t mysensor;

static void pwm_setrefint_3ph_tim1(int16_t ref[3]){
	int i;
	int32_t refi[3];
	uint32_t refu[3];

	for(i=0;i<3;i++){
		refi[i] = ( (int32_t)ref[i]);
		refi[i] += 32768;
		refu[i] = refi[i];
		refu[i] = refu[i] * PWM_MAX;
		refu[i] = refu[i] >> 16;
	}

	TIM1->CCR1 = refu[0];
	TIM1->CCR2 = refu[1];
	TIM1->CCR3 = refu[2];
}

void control_sm_to_ready_mode(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	myctrl.ctrl_sm_cnt = 0;
	myctrl.state = ctrl_sm_state_ready;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	//can_init();
}

void control_sm_to_off_mode(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	myctrl.ctrl_sm_cnt = 0;
	myctrl.state = ctrl_sm_state_off;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	//can_init();
}

void control_sm_to_error(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	myctrl.ctrl_sm_cnt = 0;
	myctrl.state = ctrl_sm_state_error;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
}

void control_sm_to_vcon_mode(void){
	myctrl.ua = 0;
	myctrl.ub = 0;
	myctrl.uc = 0;
	myctrl.state = ctrl_sm_state_vcon;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	LL_TIM_EnableAllOutputs(TIM1); //PWM on
}

void control_sm_to_ccon_mode(void){
	//Transfer to ccon mode, do initialization of controllers
	myctrl.pi_a.i_val = 0;
#ifndef SINGLECOIL_DESIGN
	myctrl.pi_b.i_val = 0;
#endif
	myctrl.ctrl_sm_cnt = 0;
	myctrl.state = ctrl_sm_state_ccon;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	LL_TIM_EnableAllOutputs(TIM1); //PWM on
}

void control_sm(void){
	if(myctrl.state == ctrl_sm_state_startup){
		myctrl.ctrl_sm_cnt++;
		//delay 0.5s
		if(myctrl.ctrl_sm_cnt >= 8000){
			myctrl.ctrl_sm_cnt = 0;
			myctrl.ia_offs_sum = myctrl.ib_offs_sum = myctrl.ic_offs_sum = 0;
			myctrl.state = ctrl_sm_state_curroffs;
		}
	}
	else if(myctrl.state == ctrl_sm_state_curroffs){
		//wait 0.5s for offset determination
		if(myctrl.ctrl_sm_cnt < 8000){
			myctrl.ctrl_sm_cnt++;
		}
		else {
			if (calib_data.i_autoOffs){
				calib_data.ia.offs = myctrl.ia_offs_sum / 8000;
				calib_data.ib.offs = myctrl.ib_offs_sum / 8000;
				calib_data.ic.offs = myctrl.ic_offs_sum / 8000;
			}
			myctrl.ctrl_sm_cnt = 0;
			control_sm_to_ready_mode();
		}
	}
	else if(myctrl.state == ctrl_sm_state_ready){
		myctrl.ctrl_sm_cnt++;
		//if(myctrl.ctrl_sm_cnt >= 8000){  //with autostart after 0.5s
		if(myctrl.ctrl_sm_cnt >= 64000){  //with autostart after 4s
			//if(button_pressed){
			control_sm_to_ccon_mode();
			//}
		}
	}
	else if(myctrl.state == ctrl_sm_state_ccon){
		if(myctrl.cmd == ctrl_sm_cmd_stop){
			myctrl.cmd = ctrl_sm_cmd_none;
			control_sm_to_off_mode();
		}
		else if(myctrl.ctrl_sm_cnt < 8000){ //Switch off via button possible after at least 0.5s
			myctrl.ctrl_sm_cnt++;
		}
		else {
			if(myctrl.button_pressed){
				control_sm_to_off_mode();
			}
		}
	}
	else if(myctrl.state == ctrl_sm_state_off){
		//Restart when button pressed
		if((myctrl.button_pressed) || myctrl.cmd == ctrl_sm_cmd_start){
			myctrl.cmd = ctrl_sm_cmd_none;
			myctrl.ctrl_sm_cnt = 0;
			control_sm_to_ready_mode();
		}
	}
	else if(myctrl.state == ctrl_sm_state_error){
		//Reset via button possible after at least 1s
		if(myctrl.ctrl_sm_cnt < 16000){
			//If button is pressed before 1s elapsed, ignore it and restart counting
			if((myctrl.button_pressed) || myctrl.cmd == ctrl_sm_cmd_start){
				myctrl.cmd = ctrl_sm_cmd_none;
				myctrl.ctrl_sm_cnt = 0;
			}
			else {
				myctrl.ctrl_sm_cnt++;
			}
		}
		else {
			//Error reset when button pressed
			if((myctrl.button_pressed) || myctrl.cmd == ctrl_sm_cmd_start){
				myctrl.cmd = ctrl_sm_cmd_none;
				myctrl.ctrl_sm_cnt = 0;
				control_sm_to_ready_mode();
			}
		}
	}
}

void sens_eval(void){
	if( mysensor.state == sens_state_wft){
		mysensor.cnt = 0;
		if(myctrl.poti < mysensor.level-mysensor.hyst){
			mysensor.state = sens_state_run;
		}
		if(mysensor.cmd == sens_cmd_trigger){
			mysensor.cmd = sens_cmd_none;
			mysensor.cnt = 0;
			mysensor.state = sens_state_rdy;
		}
	}
	else if( mysensor.state == sens_state_run){
		mysensor.cnt++;
		if(myctrl.poti > mysensor.level+mysensor.hyst){
			mysensor.spd = 1600000/mysensor.cnt;
			mysensor.pos = 0;
			mysensor.cnt = 0;
			mysensor.state = sens_state_fly;
		}
	}
	else if( mysensor.state == sens_state_fly){
		mysensor.cnt++;
		mysensor.pos += mysensor.spd;
		if(mysensor.cnt > 16000){
			mysensor.state = sens_state_wft;
		}
		if(mysensor.pos > mysensor.trigpos){
			mysensor.cnt = 0;
			mysensor.state = sens_state_rdy;
		}
	}
	else if( mysensor.state == sens_state_rdy){
		mysensor.cnt++;
		if(mysensor.cmd == sens_cmd_reset){
			mysensor.cmd = sens_cmd_none;
			mysensor.state = sens_state_wft;
		}
	}

}


void control_init(void){
	myctrl.cnt_starta = 0;
	myctrl.cnt_stopa = 200;
	myctrl.iaval = 3000; //ca. 10A

	myctrl.cnt_startb = 200;
	myctrl.cnt_stopb = 800;
	myctrl.ibval = 4200;
	//myctrl.ibval = 0;

	//Current controllers
	myctrl.pi_a.kp = 0.5 * 4*256;     // 0.5V/A
	myctrl.pi_a.ki = 0.1 * 16*256;    // 0.1V/(A*Ts) = 1.6V/(A*ms) (for Ts = 0.0625ms)
#ifndef SINGLECOIL_DESIGN
	myctrl.pi_b.kp = 0.5 * 4*256;     // 0.5V/A
	myctrl.pi_b.ki = 0.1 * 16*256;    // 0.1V/(A*Ts) = 1.6V/(A*ms) (for Ts = 0.0625ms)
#endif
	myctrl.iaref = 0;
	myctrl.ibref = 0;

	myctrl.state = ctrl_sm_state_startup;
	myctrl.cmd = ctrl_sm_cmd_none;
	myctrl.ctrl_sm_cnt = 0;

	myctrl.error = ctrl_errors_none;

	myctrl.clock.ticks_per_ms = 16;
	myctrl.clock.ticks = 0;
	myctrl.clock.ms = 0;
	myctrl.clock.s = 0;

	mysensor.cnt = 0;
	mysensor.spd = 0;
	mysensor.pos = 0;
	mysensor.trigpos = 3000000;
	mysensor.level = 3500;
	mysensor.hyst = 100;
	mysensor.state = sens_state_wft;

	LL_TIM_EnableIT_UPDATE(TIM1); //PWM interrupt
}

//32kHz PWM
//16kHz call frequency, due to RepetitionCounter = 3
void control_pwm_ctrl(void){
	myctrl.timCnt_ctrl_begin = TIM1->CNT;

	//Check for button press
	if(LL_GPIO_IsInputPinSet(BUTTON_GPIO_Port, BUTTON_Pin)){
		myctrl.button_pressed = 0;
	} else{
		myctrl.button_pressed = 1;
	}
	//Pull down pin in order to indicate function begin (can be measured by oscilloscope)
	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_DOWN);

	if(LL_GPIO_IsInputPinSet(ENC_A_GPIO_Port, ENC_A_Pin)){
		myctrl.trigger_in = 0;
	} else{
		myctrl.trigger_in = 1;
	}

	// *** ADC values read and conversion ***
	int retry = 0;
	while(!LL_ADC_IsActiveFlag_JEOS(ADC1)){
		retry++;
		//Error after certain time, if adc will not get ready -> signal error!
		//should not happen, because due to encoder read and conversion, a lot of time has passed
		if(retry >= 10){
			if (myctrl.state != ctrl_sm_state_startup){
				myctrl.error = ctrl_errors_adc;
				control_sm_to_error();
			}
		}
	}
	//Currents are in int16, Q7.8
	int16_t ia_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	myctrl.ia = interp_linearTrsfm_i16(&calib_data.ia, ia_raw);

	//1024 = 9.0V,  1143 = 10.0V, 1262 = 11.0V
	int16_t vbusraw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
	myctrl.vbus = interp_linearTrsfm_i16(&calib_data.vbus, vbusraw);

	int16_t potiraw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
	myctrl.poti = interp_linearTrsfm_i16(&calib_data.poti, potiraw);

	int16_t ib_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	myctrl.ib = interp_linearTrsfm_i16(&calib_data.ib, ib_raw);

	int16_t ic_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
	myctrl.ic = interp_linearTrsfm_i16(&calib_data.ic, ic_raw);

	// *** Error handling ***
	if(myctrl.state == ctrl_sm_state_ccon || myctrl.state == ctrl_sm_state_vcon){
		//signal over- / undervoltage and overcurrent
		if(  (myctrl.ia > calib_data.imax)  ||  (myctrl.ia < -calib_data.imax)   )  {
			myctrl.error = ctrl_errors_overcurrent;
			control_sm_to_error();
		}
		if(  (myctrl.ib > calib_data.imax)  ||  (myctrl.ib < -calib_data.imax)   )  {
			myctrl.error = ctrl_errors_overcurrent;
			control_sm_to_error();
		}
		if(  (myctrl.ic > calib_data.imax)  ||  (myctrl.ic < -calib_data.imax)   )  {
			myctrl.error = ctrl_errors_overcurrent;
			control_sm_to_error();
		}
		if(  (myctrl.vbus > calib_data.umax) ) {
			myctrl.error = ctrl_errors_overvoltage;
			control_sm_to_error();
		}
		if(  (myctrl.vbus < calib_data.umin) ) {
			myctrl.error = ctrl_errors_undervoltage;
			control_sm_to_error();
		}
	}

	// vbus inverse calculation
	int16_t vbus = myctrl.vbus;
	if( vbus < (5<<8) ) {   //minimum 5V for inverse calculation
		vbus = (5<<8);
	}
	myctrl.divVbus = 8388608/vbus;   //2^23  (.vbus is Q7.8, .divVbus is Q0.15)

	sens_eval();

	if (myctrl.state == ctrl_sm_state_curroffs) {
		myctrl.ia_offs_sum -= ia_raw;
		myctrl.ib_offs_sum -= ib_raw;
		myctrl.ic_offs_sum -= ic_raw;
	}


	// *** Current setpoint generation ***
	myctrl.iaref = 256*0; //0A
	myctrl.ibref = 256*0; //0A

	if(myctrl.state == ctrl_sm_state_ccon){
		if(mysensor.state == sens_state_rdy){
			if(   (mysensor.cnt > myctrl.cnt_starta) && (mysensor.cnt < myctrl.cnt_stopa)){
				myctrl.iaref = myctrl.iaval; //for testing
			}
			if(   (mysensor.cnt > myctrl.cnt_startb) && (mysensor.cnt < myctrl.cnt_stopb)){
				myctrl.ibref = myctrl.ibval; //for testing
			}
			if(mysensor.cnt > myctrl.cnt_stopb){
				mysensor.cmd = sens_cmd_reset;
			}
		}
	}

	// *** Current ***
	if( myctrl.state == ctrl_sm_state_ccon ){
		//Current Controllers
		myctrl.pi_a.max = myctrl.vbus;
		myctrl.pi_a.min = -myctrl.vbus;
#ifndef SINGLECOIL_DESIGN
		myctrl.pi_b.max = myctrl.vbus;
		myctrl.pi_b.min = -myctrl.vbus;
#endif

#ifdef SINGLECOIL_DESIGN
		myctrl.ua = (control_pictrl_i16(&myctrl.pi_a,myctrl.iaref,myctrl.ia) );
		myctrl.uc = -myctrl.ua;
#else

		myctrl.ua = control_pictrl_i16(&myctrl.pi_a,myctrl.iaref,myctrl.ia);
		myctrl.ub = control_pictrl_i16(&myctrl.pi_b,myctrl.ibref,myctrl.ib);
		myctrl.uc = 0;

		//Symmetrizing
		int32_t uavg = (myctrl.ua + myctrl.ub + myctrl.uc) / 3;
		int16_t uavg16 = (int16_t)uavg;
		myctrl.ua -= uavg16;
		myctrl.ub -= uavg16;
		myctrl.uc -= uavg16;
#endif
	}
	else {
		myctrl.ua = myctrl.ub = myctrl.uc = 0;
	}

	int32_t tmpref = (myctrl.ua*myctrl.divVbus) >> 8;  //Q7.8 * Q0.15 >> 8 = ???
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	myctrl.pwmrefa = tmpref;

	tmpref = (myctrl.ub*myctrl.divVbus) >> 8;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	myctrl.pwmrefb = tmpref;

	tmpref = (myctrl.uc*myctrl.divVbus) >> 8;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	myctrl.pwmrefc = tmpref;

	if(myctrl.state == ctrl_sm_state_ready){
		myctrl.pwmrefa = 0;
		myctrl.pwmrefb = 0;
		myctrl.pwmrefc = 0;
	}

	//PWM output
	int16_t refs[3];
	refs[0] = myctrl.pwmrefa;
	refs[1] = myctrl.pwmrefb;
	refs[2] = myctrl.pwmrefc;
	pwm_setrefint_3ph_tim1(refs);

	myctrl.timCnt_ctrl_check = TIM1->CNT;

	control_sm();

	datarec_sm();
	//can_sm();


	myctrl.clock.ticks++;
	if(myctrl.clock.ticks >= myctrl.clock.ticks_per_ms){
		myctrl.clock.ticks=0;
		myctrl.clock.ms++;
	}
	if(myctrl.clock.ms >= 1000){
		myctrl.clock.ms=0;
		myctrl.clock.s++;
	}

	//Disable pull down in order to indicate function end
	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_NO);

	myctrl.timCnt_ctrl_end = TIM1->CNT;
}


// **** Code related to Data Recording ****
#define BUFLEN 400 //Buffer length
#define CYCLEDIV 4 //cycle div

typedef enum {
	datarec_state_startup = 0,
	datarec_state_wft = 1,
	datarec_state_rec = 2,
	datarec_state_rdy = 3
} datarec_states_t;


static struct{
	int16_t ia_buf[BUFLEN];
	int16_t ib_buf[BUFLEN];
	int16_t ic_buf[BUFLEN];
	int16_t ua_buf[BUFLEN];
	int16_t vbus_buf[BUFLEN];
} myDataRecData;


static int i_elecnt = 0;
static int dataRecTrig = 0;
static datarec_states_t myDataRecState = datarec_state_startup;

void datarec_sm(void){
	if(myDataRecState == datarec_state_startup){
		myDataRecState = datarec_state_wft;
	}
	else if(myDataRecState == datarec_state_wft){
		if(dataRecTrig){
			i_elecnt = 0;
			myDataRecState = datarec_state_rec;
		}
	}
	else if(myDataRecState == datarec_state_rec){
		//store debug data to buffers
		int i_elecnt_dn = i_elecnt / CYCLEDIV;
		myDataRecData.ia_buf[i_elecnt_dn] = myctrl.ia;
		myDataRecData.ib_buf[i_elecnt_dn] = myctrl.ib;
		myDataRecData.ic_buf[i_elecnt_dn] = myctrl.ic;
		myDataRecData.ua_buf[i_elecnt_dn] = myctrl.ua;
		myDataRecData.vbus_buf[i_elecnt_dn] = myctrl.vbus;

		i_elecnt++;
		if(i_elecnt/CYCLEDIV >= BUFLEN) {
			myDataRecState = datarec_state_rdy;
		}
	}
	else if(myDataRecState == datarec_state_rdy){
		if(dataRecTrig == 0){ //wait for manual restart (use debugger!!)
			myDataRecState = datarec_state_wft;
		}
	}
}


// **** Code related to CAN ****
#ifdef USE_CAN

static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];
static FDCAN_TxHeaderTypeDef TxHeader;
static uint8_t TxData[8];


static volatile struct{
	int cnt;
	int runs;
	int errors;
} myCanData;

void can_init(void){
	myCanData.cnt = 0;
	myCanData.runs = 0;
	myCanData.errors = 0;
}

void can_sm(void){
	//if(mysens.state != sens_state_wft){ //send CAN only when active

	myCanData.cnt++;
	if(myCanData.cnt >= 16){ //1kHz
	//if(myCanData.cnt >= 4){ //4kHz
		myCanData.cnt = 0;
		//TxData[0] = 0x34;
		//TxData[1]++;
		TxData[0] = (myctrl.ua) / 64;
		TxData[1] = (myctrl.ub) / 64;
		TxData[2] = (myctrl.ia) / 64;
		TxData[3] = (myctrl.ib) / 64;
		myCanData.runs++;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		{
			/* Transmission request Error */
			myCanData.errors++;
			//Error_Handler();
		}
	}
}


void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK; //FDCAN_FILTER_RANGE, FDCAN_FILTER_DUAL ...
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x321;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;


  //LL_GPIO_ResetOutputPin(CAN_TERM_GPIO_Port, CAN_TERM_Pin);
  //LL_GPIO_ResetOutputPin(CAN_SHDN_GPIO_Port, CAN_SHDN_Pin);

  LL_GPIO_SetOutputPin(CAN_TERM_GPIO_Port, CAN_TERM_Pin); //Enable 120 Ohm terminating Resistor
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    Error_Handler();
    }

    /* Display LEDx */
    if ((RxHeader.Identifier == 0x321) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_2))
    {
      //LED_Display(RxData[0]);
      //ubKeyNumber = RxData[0];
    }
  }
}

#endif //USE_CAN
