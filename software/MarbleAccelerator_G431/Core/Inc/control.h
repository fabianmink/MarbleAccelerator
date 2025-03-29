#include "main.h"

//#define SINGLECOIL_DESIGN


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




