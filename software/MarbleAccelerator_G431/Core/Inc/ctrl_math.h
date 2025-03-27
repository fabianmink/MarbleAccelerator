#include <stdint.h>

typedef struct
{
	int16_t kp;       //  P-Gain Q5.10
	int16_t ki;       //  I-Gain Q3.12
	int16_t min;      //  lower output limit
	int16_t max;      //  upper output limit
	int32_t i_val;    //  current integral value
} control_pictrl_i16_t;


typedef struct
{
	int16_t gain;        //  gain, Q7.8
	int16_t offs;        //  offset
} interp_lin_i16_t;


extern int16_t control_pictrl_i16(control_pictrl_i16_t* controller, int16_t ref, int16_t act);
extern int16_t interp_linearTrsfm_i16(interp_lin_i16_t* coeffs, int16_t in);

