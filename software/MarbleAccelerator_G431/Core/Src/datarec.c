#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "control.h"
#include "datarec.h"


// **** Code related to Data Recording ****
#define BUFLEN 16*50 //Buffer length, 50ms

typedef enum {
	datarec_state_startup = 0,
	datarec_state_wft = 1,
	datarec_state_rec = 2,
	datarec_state_rdy = 3
} datarec_states_t;

typedef struct  {
	char* name;
	size_t cnt;
	int16_t* pdata;
} datarec_dataElement_t;


typedef struct {
	size_t cnt;
	datarec_dataElement_t* elements[];
} datarec_dataList_t;

static int16_t ia_buf[BUFLEN];
static int16_t ib_buf[BUFLEN];
static int16_t ic_buf[BUFLEN];
static int16_t ua_buf[BUFLEN];
static int16_t ub_buf[BUFLEN];
static int16_t uc_buf[BUFLEN];
static int16_t vbus_buf[BUFLEN];

static datarec_dataElement_t ele_ia = {
		.name = "ia",
		.cnt = BUFLEN,
		.pdata = ia_buf
};

static datarec_dataElement_t ele_ib = {
		.name = "ib",
		.cnt = BUFLEN,
		.pdata = ib_buf
};

static datarec_dataList_t myList = {
		.cnt = 2,
		.elements = {
				[0] = &ele_ia,
				[1] = &ele_ib,
		}
};


typedef struct{
	datarec_states_t state;
	datarec_dataElement_t* pdata;
	int downcycle;
	int cntData;
	int cntDowncycle;
	bool trigger;
} datarec_t;


static datarec_t myDatarec;


void datarec_init(void){

}

void datarec_sm(void){
	if(myDatarec.state == datarec_state_startup){
		myDatarec.state = datarec_state_wft;
	}
	else if(myDatarec.state == datarec_state_wft){
		if(myDatarec.trigger){
			myDatarec.cntData = 0;
			myDatarec.cntDowncycle = myDatarec.downcycle;
			myDatarec.state = datarec_state_rec;
		}
	}
	else if(myDatarec.state == datarec_state_rec){
		//store  data to buffers
		if(myDatarec.cntDowncycle > 0){
			myDatarec.cntDowncycle--;
		}
		else {
			ia_buf[myDatarec.cntData] = myctrl.ia;
			ib_buf[myDatarec.cntData] = myctrl.ib;
			ic_buf[myDatarec.cntData] = myctrl.ic;
			ua_buf[myDatarec.cntData] = myctrl.ua;
			ub_buf[myDatarec.cntData] = myctrl.ub;
			uc_buf[myDatarec.cntData] = myctrl.uc;
			vbus_buf[myDatarec.cntData] = myctrl.vbus;

			myDatarec.cntDowncycle=myDatarec.downcycle;
			myDatarec.cntData++;

			if(myDatarec.cntData >= BUFLEN) {
				myDatarec.state = datarec_state_rdy;
			}
		}

	}
	else if(myDatarec.state == datarec_state_rdy){
		if(myDatarec.trigger == 0){ //wait for manual restart (use debugger!!)
			myDatarec.state = datarec_state_wft;
		}
	}
}

#include "stm32g4xx_hal.h"
static void datarec_print(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit_IT(huart, pData, Size);
	//Wait until complete, as HAL_UART_Transmit_IT doesn't seem to copy the pData contents
	while(   HAL_UART_GetState(huart) != HAL_UART_STATE_READY ){
	}
}


static void datarec_printstr(UART_HandleTypeDef *huart, char *str){
	size_t len;
	len = strlen(str);
	datarec_print(huart, (uint8_t*)str, len);
}

void datarec_printResults(UART_HandleTypeDef *huart){
	datarec_printstr(huart, "{");

	for(int listidx = 0; listidx<myList.cnt; listidx++){
		char* eleName;
		datarec_dataElement_t* ele = myList.elements[listidx];
		eleName = ele->name;

		if(listidx > 0)	datarec_printstr(huart, "],\r\n\"");
		else datarec_printstr(huart, "\r\n\"");

		datarec_printstr(huart, eleName);
		datarec_printstr(huart, "\": [");

		size_t numele = ele->cnt;
		for(int idx = 0; idx<numele; idx++){
			char sval[10];
			itoa(ele->pdata[idx], sval, 10); //todo: Maybe change base to 16
			if(idx > 0) datarec_printstr(huart, ", ");
			datarec_printstr(huart, sval);
		}
	}
	datarec_printstr(huart, "]\r\n}\r\n");
}
