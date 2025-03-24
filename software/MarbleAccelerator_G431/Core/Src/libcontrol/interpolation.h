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


#ifndef __INTERPOLATION_H
#define __INTERPOLATION_H

typedef struct
{
	int nb;              // number of data points
	unsigned char mode;  // extrapolation mode
	float* b;            // pointer to data points, array size must fit nb (strictly increasing!!!!)
	float* val;          // pointer to data values, array size must fit nb
} lin1_data_t;


typedef struct
{
	int nbx;             // number of data points, x-direction
	int nby;             // number of data points, y-direction
	unsigned char mode;  // extrapolation mode
	float* bx;           // pointer to x-data points, array size must fit nbx (strictly increasing!!!!)
	float* by;           // pointer to y-data points, array size must fit nby (strictly increasing!!!!)
	float* val;          // pointer to data values, array size must be nbx * nby
} lin2_data_t;

#endif /* __INTERPOLATION_H */
