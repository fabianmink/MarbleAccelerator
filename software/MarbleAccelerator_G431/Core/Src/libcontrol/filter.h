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

#ifndef __FILTER_H
#define __FILTER_H

typedef struct
{
  float kf;     //  Filter constant
  float out;    //  Filter last output = internal state
} iir1_filter_t;


//1st order IIR (PT1) Filter, design by time constant
extern int filter_iir1_init(iir1_filter_t* filter, float tau, float Ts);
//1st order IIR (PT1) Filter with unity gain, design by cutoff frequency
extern int filter_iir1_init_fc(iir1_filter_t* filter, float fc, float Ts);

//1st order IIR (PT1) Filter, reset to certain value
extern void filter_iir1_reset(iir1_filter_t* filter, float val);
//1st order IIR (PT1) Filter
extern float filter_iir1(iir1_filter_t* filter, float in);


#endif /* __FILTER_H */