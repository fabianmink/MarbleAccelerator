/*
  Copyright (c) 2015 Fabian Mink <fabian.mink@gmx.de>
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

#include <stdint.h>

#ifndef __FIFO_H
#define __FIFO_H

typedef struct {
	uint8_t* buffer;
	int size;
	volatile int readPointer;
	volatile int writePointer;
} uint8Fifo_t;


extern int fifo_uint8Write(uint8Fifo_t* fifo, uint8_t* value);
extern int fifo_uint8Read(uint8Fifo_t* fifo, uint8_t* value);
extern int fifo_uint8IsEmpty(uint8Fifo_t* fifo);
extern int fifo_uint8IsFull(uint8Fifo_t* fifo);
extern int fifo_uint8GetFill(uint8Fifo_t* fifo);
extern int fifo_uint8GetRemain(uint8Fifo_t* fifo);
extern int fifo_uint8Clear(uint8Fifo_t* fifo);

#endif //__FIFO_H
