/*
  Copyright (c) 2020 Fabian Mink <fabian.mink@gmx.de>
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

#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

typedef struct
{
  float kp;       //  P-Gain
  float ki;       //  I-Gain
  float min;      //  lower output limit
  float max;      //  upper output limit
  float i_val;    //  current integral value
} pictrl_t;


typedef struct
{
  float kp;       //  P-Gain
  float kd;       //  D-Gain
  float min;      //  lower output limit
  float max;      //  upper output limit
  float diff_n1;  //  last ctrldiff value
} pdctrl_t;

typedef struct
{
  float kp;       //  P-Gain
  float kd;       //  D-Gain
  iir1_filter_t filter;  //  1st order filter   
  float min;      //  lower output limit
  float max;      //  upper output limit
  float diff_n1;  //  last ctrldiff value  
} pdt1ctrl_t;

typedef struct
{
  float kp;       //  P-Gain
  float ki;       //  I-Gain
  float kd;       //  D-Gain
  iir1_filter_t filter;  //  1st order filter
  float min;      //  lower output limit
  float max;      //  upper output limit
  float i_val;    //  current integral value
  float diff_n1;  //  last ctrldiff value
} pidt1ctrl_t;

extern int control_pictrl_init(pictrl_t* controller, float Kp, float Ti, float Ts);
extern float control_pictrl(pictrl_t* controller, float ref, float act);

extern int control_pdctrl_init(pdctrl_t* controller, float Kp, float Td, float Ts);
extern float control_pdctrl(pdctrl_t* controller, float ref, float act);

extern int control_pdt1ctrl_init(pdt1ctrl_t* controller, float Kp, float Td, float T1, float Ts);
extern float control_pdt1ctrl(pdt1ctrl_t* controller, float ref, float act);

extern int control_pidt1ctrl_init(pidt1ctrl_t* controller, float Kp, float Ti, float Td, float T1, float Ts);
extern float control_pidt1ctrl(pidt1ctrl_t* controller, float ref, float act);
#endif /* __CONTROL_H */
