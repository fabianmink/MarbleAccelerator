#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

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



#define WFB_SYNC_TIMER_PERIOD    50000   //10ms
#define WFB_SYNC_REF             8600    //@ 1.72ms = 50000 * 0.172

#define WFB_SLAVE_TX_POS_OFFS    10000   //@ 2.0ms = 50000 * 0.20, Offset for slave 0
#define WFB_SLAVE_TX_POS_SHIFT   10000   //@ 2.0ms, every slave is shifted by this value

#define WFB_MASTER_DATA_BYTES_BASIC       2  // 1 general command + 1 service
#define WFB_MASTER_DATA_BYTES_PER_SLAVE   5  // 1 individual command + 4 position
#define WFB_MASTER_DATA_BYTES_HMAC        4

#define INSYNC_CNT_DIFF         250     //50us = 50000 * 0.005
#define INSYNC_CNT_LEN          20
#define INSYNC_MAX_LOST_FRAMES  3


