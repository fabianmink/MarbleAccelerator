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


//#include <math.h>
//#include <string.h>

#include "control.h"

#define PWM_MAX 2500



//Power stage XXX
static calib_data_t calib_data = {
		.ia = {.offs = 0, .gain = -1900},
		.ib = {.offs = 0, .gain = -1900},
		.ic = {.offs = 0, .gain = -1900},
		.i_autoOffs = true,
		.vbus = {.offs = 47, .gain = 551},
		.poti = {.offs = 0,  .gain = 256}
};

static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];
static FDCAN_TxHeaderTypeDef TxHeader;
static uint8_t TxData[8];


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


typedef struct
{
	int16_t kp;       //  P-Gain Q13.2
	int16_t ki;       //  I-Gain Q7.8
	int16_t min;      //  lower output limit
	int16_t max;      //  upper output limit
	int32_t i_val;    //  current integral value
} control_pictrl_i16_t;


int16_t control_pictrl_i16(control_pictrl_i16_t* controller, int16_t ref, int16_t act){

	int32_t ctrldiff;
	int32_t p_val, i_val;
	int32_t tmp_out;
	//int16_t out;

	ctrldiff = ((int32_t)ref - (int32_t)act);
	p_val = (((int32_t)controller->kp) * ctrldiff) >> 2;
	if(controller->ki){
		i_val = ((((int32_t)controller->ki) * ctrldiff)>>8) + controller->i_val;
	}
	else {
		i_val = 0.0f;
	}

	tmp_out = (p_val + i_val);

	if(tmp_out > controller->max){
		tmp_out = controller->max;
		if(i_val > controller->i_val) i_val = controller->i_val;
		//i_val = controller->max - p_val;
	}
	if(tmp_out < controller->min){
		tmp_out = controller->min;
		if(i_val < controller->i_val) i_val = controller->i_val;
		//i_val = controller->min - p_val;
	}

	controller->i_val = i_val;
	return((int16_t)tmp_out);
}


//Linear scale / offset:  out = gain*(in + offs)
int16_t interp_linearTrsfm_i16(interp_lin_i16_t* coeffs, int16_t in){
	int32_t accu;

	accu = in + coeffs->offs;
	accu *= coeffs->gain;

	return( (int16_t) (accu>>8));
}

//Control data struct
typedef struct{
	int16_t ia;
	int16_t ib;
	int16_t ic;
	int16_t ua;
	int16_t ub;
	int16_t uc;
	int16_t vbus;
	int16_t iaref;
	int16_t iaval;
	int16_t ibref;
	int16_t ibval;
	//uint32_t cnt_ccon;
	//uint32_t cnt_period;
	//uint32_t cnt_reTrig;
	uint32_t cnt_starta;
	uint32_t cnt_stopa;
	uint32_t cnt_startb;
	uint32_t cnt_stopb;
	int16_t poti;
} ctrl_data_t;
static ctrl_data_t myctrl;

static uint8_t imax = 20;
static uint8_t umax = 25;
static uint8_t umin = 8;
static int32_t ia_offs_sum;
static int32_t ib_offs_sum;
static int32_t ic_offs_sum;


typedef enum {
	sens_state_wft = 0,
	sens_state_run = 1, //inside sensor windows
	sens_state_fly = 2, //behind sensor windows, "free flying"
	sens_state_rdy = 3  //in coil
} sens_states_t;

typedef enum {
	sens_cmd_none = 0,
	sens_cmd_reset = 1
}
sens_cmd_t;

typedef struct{
	uint32_t cnt;
	uint16_t spd;
	uint32_t pos;
	uint32_t trigpos;
	uint16_t level;
	uint16_t hyst;
	sens_states_t state;
	sens_cmd_t cmd;
} sens_data_t;
static sens_data_t mysens = {
		.cnt = 0,
		.spd = 0,
		.pos = 0,
		.trigpos = 6000000,
		.level = 3500,
		.hyst = 100,
		.state = sens_state_wft
};


static bool button_pressed = 0;  //indicate a button press
static bool trigger_in = 0;  //indicate trigger event

//Data recorder (debugging)
typedef enum {
	datarec_state_startup = 0,
	datarec_state_wft = 1,
	datarec_state_rec = 2,
	datarec_state_rdy = 3
} datarec_states_t;


#define BUFLEN 400 //Buffer length
#define CYCLEDIV 4 //cycle div

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
	//}

	}
}


typedef enum {
	ctrl_sm_state_startup = 0,
	ctrl_sm_state_curroffs = 1,
	ctrl_sm_state_ready = 3,
	ctrl_sm_state_ccon = 4,
	ctrl_sm_state_error = 8
} ctrl_sm_states_t;

//Current controllers
static control_pictrl_i16_t pi_a = {
		.i_val = 0,
		.ki = 40,
		.kp = 4,
		.max = 0,  // will be adapted "online" to vbus!!
		.min = 0
};

static control_pictrl_i16_t pi_b = {
		.i_val = 0,
		.ki = 40,
		.kp = 4,
		.max = 0,
		.min = 0
};


static ctrl_sm_states_t ctrl_sm_state = ctrl_sm_state_startup;
static int ctrl_sm_cnt = 0;
static int ctrl_sm_edge_detect = 0;

void control_sm_to_ready_mode(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	ctrl_sm_state = ctrl_sm_state_ready;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	ctrl_sm_edge_detect = 0;

	can_init();
}

void control_sm_to_error(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	ctrl_sm_cnt = 0;
	ctrl_sm_state = ctrl_sm_state_error;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	ctrl_sm_edge_detect = 0;
}

void control_sm_to_ccon_mode(void){
	//Transfer to pos mode, do initialization
	pi_a.i_val = 0;
	pi_b.i_val = 0;
	//pi_q.i_val = 0;
	//myctrl.cnt_ccon = 0;
	ctrl_sm_state = ctrl_sm_state_ccon;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	LL_TIM_EnableAllOutputs(TIM1); //PWM on
	ctrl_sm_edge_detect = 0;
}


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


void control_sm(void){

	//master_data[master_data_pread] is always consistent, because WFB ISR can not interrupt control ISR

	if(ctrl_sm_state == ctrl_sm_state_startup){
		ctrl_sm_cnt++;
		//delay 0.5s
		if(ctrl_sm_cnt >= 8000){
			ctrl_sm_cnt = 0;
			ia_offs_sum = ib_offs_sum = ic_offs_sum = 0;
			ctrl_sm_state = ctrl_sm_state_curroffs;

			//myctrl.cnt_period = 160000; //10s
			//myctrl.cnt_reTrig = 1600;

			mysens.trigpos = 3000000;
			myctrl.cnt_starta = 0;
			myctrl.cnt_stopa = 200;
			myctrl.iaval = 3000; //ca. 10A

			myctrl.cnt_startb = 200;
			myctrl.cnt_stopb = 800;
			myctrl.ibval = 4200;
			//myctrl.ibval = 0;
		}
	}
	else if(ctrl_sm_state == ctrl_sm_state_ready){
		ctrl_sm_cnt++;
		//if(button_pressed){
		if(ctrl_sm_cnt >= 8000){  //with autostart after 0.5s
			control_sm_to_ccon_mode();
		}
		else{
			ctrl_sm_edge_detect = 1;
		}
	}
	else if(ctrl_sm_state == ctrl_sm_state_error){
		ctrl_sm_cnt++;
		if(ctrl_sm_cnt >= 80000){  //with autostart after 5s
			control_sm_to_ready_mode();
		}
		else{
			ctrl_sm_edge_detect = 1;
		}
	}
	else if(ctrl_sm_state == ctrl_sm_state_curroffs){
		ctrl_sm_cnt++;
		//wait 0.5s for offset determination
		if((!calib_data.i_autoOffs) || (ctrl_sm_cnt >= 8000)){
			control_sm_to_ready_mode();
			if (calib_data.i_autoOffs){
				calib_data.ia.offs = ia_offs_sum / 8000;
				calib_data.ib.offs = ib_offs_sum / 8000;
				calib_data.ic.offs = ic_offs_sum / 8000;
			}
			//Optionally, do not do this automatically, but only after command from master
			ctrl_sm_cnt = 0;
			control_sm_to_ready_mode();
		}

	}
	else if(ctrl_sm_state == ctrl_sm_state_ccon){
		ctrl_sm_cnt = 0;
		if(button_pressed){
			if(ctrl_sm_edge_detect){
				control_sm_to_ready_mode();
			}
		}
		else{
			ctrl_sm_edge_detect = 1;
		}
	}

}

void sens_eval(void){
	if( mysens.state == sens_state_wft){
		mysens.cmd = sens_cmd_none;
		mysens.cnt = 0;
		if(myctrl.poti < mysens.level-mysens.hyst){
			mysens.state = sens_state_run;
		}
	}
	else if( mysens.state == sens_state_run){
		mysens.cnt++;
		if(myctrl.poti > mysens.level+mysens.hyst){
			mysens.spd = 1600000/mysens.cnt;
			mysens.pos = 0;
			mysens.cnt = 0;
			mysens.state = sens_state_fly;
		}
	}
	else if( mysens.state == sens_state_fly){
		mysens.cnt++;
		mysens.pos += mysens.spd;
		if(mysens.cnt > 16000){
			mysens.state = sens_state_wft;
		}
		if(mysens.pos > mysens.trigpos){
			mysens.cnt = 0;
			mysens.state = sens_state_rdy;
		}
	}
	else if( mysens.state == sens_state_rdy){
		mysens.cnt++;
		if(mysens.cmd == sens_cmd_reset){
			mysens.state = sens_state_wft;
		}
	}

}

//32kHz PWM
//16kHz call frequency, due to RepetitionCounter = 3
void control_pwm_ctrl(void){
	if(LL_GPIO_IsInputPinSet(BUTTON_GPIO_Port, BUTTON_Pin)){
		button_pressed = 0;
	} else{
		button_pressed = 1;
	}
	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_DOWN);

	if(LL_GPIO_IsInputPinSet(ENC_A_GPIO_Port, ENC_A_Pin)){
		trigger_in = 0;
	} else{
		trigger_in = 1;
	}


	control_sm();

	// *** CURRENT MEASUREMENT and transformation ***
	while(!LL_ADC_IsActiveFlag_JEOS(ADC1)){
		//Error after certain time, if adc will not get ready -> signal error!
		//should not happen, because due to encoder read and conversion, a lot of time has passed
	}

	//Currents are in int16, Q7.8
	int16_t ia_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	myctrl.ia = interp_linearTrsfm_i16(&calib_data.ia, ia_raw);

	//1024 = 9.0V,  1143 = 10.0V, 1262 = 11.0V
	int16_t vbusraw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
	myctrl.vbus = interp_linearTrsfm_i16(&calib_data.vbus, vbusraw);
	if( myctrl.vbus < (5<<8) ) {
		myctrl.vbus = (5<<8);
	}
	if( myctrl.vbus > (40<<8) ){
		myctrl.vbus = (40<<8);
	}
	int16_t divvbus = 32767/myctrl.vbus;

	int16_t potiraw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
	myctrl.poti = interp_linearTrsfm_i16(&calib_data.poti, potiraw);

	int16_t ib_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	myctrl.ib = interp_linearTrsfm_i16(&calib_data.ib, ib_raw);

	int16_t ic_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
	myctrl.ic = interp_linearTrsfm_i16(&calib_data.ic, ic_raw);

	if(ctrl_sm_state >= ctrl_sm_state_ready){
		//signal over- / undervoltage and overcurrent
		if(     (myctrl.ia > ((int16_t)imax)*256)  ||    (myctrl.ia < ((int16_t)imax)*-256)       ){
			control_sm_to_error();
		}
		if(     (myctrl.ib > ((int16_t)imax)*256)  ||    (myctrl.ib < ((int16_t)imax)*-256)       ){
			control_sm_to_error();
		}
		if(     (myctrl.ib > ((int16_t)imax)*256)  ||    (myctrl.ib < ((int16_t)imax)*-256)       ){
			control_sm_to_error();
		}
		if(     (myctrl.vbus > ((int16_t)umax)*256)  ||    (myctrl.vbus < ((int16_t)umin)*256)       ){
			control_sm_to_error();
		}
	}

	sens_eval();

	if (ctrl_sm_state == ctrl_sm_state_curroffs) {
		ia_offs_sum -= ia_raw;
		ib_offs_sum -= ib_raw;
		ic_offs_sum -= ic_raw;
	}


	// *** CONTROLLERS: Position / Current ***
	if(ctrl_sm_state == ctrl_sm_state_ccon){
//		myctrl.cnt_ccon++;
//		if( myctrl.cnt_ccon >= myctrl.cnt_period ){
//			myctrl.cnt_ccon=0;
//			dataRecTrig = 1;
//		}
//		else if( (myctrl.cnt_ccon >= myctrl.cnt_reTrig) &&  trigger_in){
//			myctrl.cnt_ccon=0;
//			dataRecTrig = 1;
//		}


		myctrl.iaref = 256*0; //0A
		myctrl.ibref = 256*0; //0A


		if(mysens.state == sens_state_rdy){
			if(   (mysens.cnt > myctrl.cnt_starta) && (mysens.cnt < myctrl.cnt_stopa)){
				myctrl.iaref = myctrl.iaval; //for testing
			}
			if(   (mysens.cnt > myctrl.cnt_startb) && (mysens.cnt < myctrl.cnt_stopb)){
				myctrl.ibref = myctrl.ibval; //for testing
			}
			if(mysens.cnt > myctrl.cnt_stopb){
				mysens.cmd = sens_cmd_reset;
			}
		}



	}

	if( ctrl_sm_state == ctrl_sm_state_ccon ){
		//Current Controllers
		pi_a.max = myctrl.vbus/2;
		pi_a.min = -myctrl.vbus/2;
		pi_b.max = myctrl.vbus/2;
		pi_b.min = -myctrl.vbus/2;

		myctrl.ua = control_pictrl_i16(&pi_a,myctrl.iaref,myctrl.ia);
		myctrl.ub = control_pictrl_i16(&pi_b,myctrl.ibref,myctrl.ib);
		myctrl.uc = -(myctrl.ua/2 + myctrl.ub/2);
	}


	int16_t refs[3];
	int32_t tmpref = myctrl.ua*divvbus;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	refs[0] = tmpref;

	tmpref = myctrl.ub*divvbus;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	refs[1] = tmpref;

	tmpref = myctrl.uc*divvbus;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	refs[2] = tmpref;

	if(ctrl_sm_state == ctrl_sm_state_ready){
		refs[0] = 0;
		refs[1] = 0;
		refs[2] = 0;
	}

	//PWM output
	pwm_setrefint_3ph_tim1(refs);

	datarec_sm();
	can_sm();

	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_NO);
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

