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
static int16_t iaref_buf[BUFLEN];
static int16_t ibref_buf[BUFLEN];
static int16_t icref_buf[BUFLEN];
static int16_t ua_buf[BUFLEN];
static int16_t ub_buf[BUFLEN];
static int16_t uc_buf[BUFLEN];
static int16_t udc_buf[BUFLEN];

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

static datarec_dataElement_t ele_ic = {
		.name = "ic",
		.cnt = BUFLEN,
		.pdata = ic_buf
};


static datarec_dataElement_t ele_iaref = {
		.name = "iaref",
		.cnt = BUFLEN,
		.pdata = iaref_buf
};

static datarec_dataElement_t ele_ibref = {
		.name = "ibref",
		.cnt = BUFLEN,
		.pdata = ibref_buf
};

static datarec_dataElement_t ele_icref = {
		.name = "icref",
		.cnt = BUFLEN,
		.pdata = icref_buf
};


static datarec_dataElement_t ele_ua = {
		.name = "ua",
		.cnt = BUFLEN,
		.pdata = ua_buf
};

static datarec_dataElement_t ele_ub = {
		.name = "ub",
		.cnt = BUFLEN,
		.pdata = ub_buf
};

static datarec_dataElement_t ele_uc = {
		.name = "uc",
		.cnt = BUFLEN,
		.pdata = uc_buf
};

static datarec_dataElement_t ele_udc = {
		.name = "udc",
		.cnt = BUFLEN,
		.pdata = udc_buf
};

static datarec_dataList_t myList = {
		.cnt = 10,
		.elements = {
				[0] = &ele_ia,
				[1] = &ele_ib,
				[2] = &ele_ic,
				[3] = &ele_iaref,
				[4] = &ele_ibref,
				[5] = &ele_icref,
				[6] = &ele_ua,
				[7] = &ele_ub,
				[8] = &ele_uc,
				[9] = &ele_udc,
		}
};


typedef struct{
	datarec_states_t state;
	datarec_dataElement_t* pdata;
	int downcycle;
	int cntData;
	int cntDowncycle;
	bool trigger;
	bool resultsRead;
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
			iaref_buf[myDatarec.cntData] = myctrl.iaref;
			ibref_buf[myDatarec.cntData] = myctrl.ibref;
			icref_buf[myDatarec.cntData] = 0;
			ua_buf[myDatarec.cntData] = myctrl.ua;
			ub_buf[myDatarec.cntData] = myctrl.ub;
			uc_buf[myDatarec.cntData] = myctrl.uc;
			udc_buf[myDatarec.cntData] = myctrl.vbus;

			myDatarec.cntDowncycle=myDatarec.downcycle;
			myDatarec.cntData++;

			if(myDatarec.cntData >= BUFLEN) {
				myDatarec.state = datarec_state_rdy;
				myDatarec.resultsRead = 0;
			}
		}

	}
	else if(myDatarec.state == datarec_state_rdy){
		if(myDatarec.resultsRead == 1){
			myDatarec.state = datarec_state_wft;
			myDatarec.trigger = 0;
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

void datarec_trigger(void){
	myDatarec.trigger = 1;
}

int datarec_isNewData(void){
	if( (myDatarec.state == datarec_state_rdy) && (myDatarec.resultsRead == 0) ) return(1);
	return(0);
}

void datarec_printResults(UART_HandleTypeDef *huart){
	datarec_printstr(huart, "{");

	for(int listidx = 0; listidx<myList.cnt; listidx++){
		char* eleName;
		datarec_dataElement_t* ele = myList.elements[listidx];
		eleName = ele->name;

		if(listidx > 0)	datarec_printstr(huart, "],\"");
		else datarec_printstr(huart, "\"");

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
	datarec_printstr(huart, "]}\r\n");

	myDatarec.resultsRead = 1;
}
