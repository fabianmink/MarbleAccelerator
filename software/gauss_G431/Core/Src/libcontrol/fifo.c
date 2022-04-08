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

#include "fifo.h"

int fifo_uint8Write(uint8Fifo_t* fifo, uint8_t* value){
	int fill = (fifo->writePointer - fifo->readPointer);

	if(fill < 0) fill += (fifo->size);
	if(fill >= fifo->size) fill -= (fifo->size);

	if(fill == (fifo->size-1)) return(-1); //Fifo full (one element remains unused)

	fifo->buffer[fifo->writePointer] = *value;

	//Write pointer inc
	fifo->writePointer++;
	if(fifo->writePointer == fifo->size) fifo->writePointer = 0;

	//(fifo->size-1): Max number of elements
	//fill: grade of filling (currently written element not counted)
	return((fifo->size-1)-fill-1); //no of available elements
}

int fifo_uint8Read(uint8Fifo_t* fifo, uint8_t* value){
	int fill = (fifo->writePointer - fifo->readPointer);

	if(fill < 0) fill += (fifo->size);
	if(fill >= fifo->size) fill -= (fifo->size);

	if(fill == 0) return(-1); //Fifo empty

	//read val
	*value = fifo->buffer[fifo->readPointer];

	//Read pointer inc
	fifo->readPointer++;
	if(fifo->readPointer == fifo->size) fifo->readPointer = 0;

	return(fill-1);
}

int fifo_uint8IsEmpty(uint8Fifo_t* fifo){
	int fill = (fifo->writePointer - fifo->readPointer);
	if(fill >= fifo->size) fill -= (fifo->size);
	if(fill == 0) return(1);

	return(0);
}

int fifo_uint8IsFull(uint8Fifo_t* fifo){
	int fill = (fifo->writePointer - fifo->readPointer);
	if(fill < 0) fill += (fifo->size);
	if(fill == (fifo->size-1)) return(1); //Fifo full (one element remains unused)

	return(0);
}

int fifo_uint8GetFill(uint8Fifo_t* fifo){
	int fill = (fifo->writePointer - fifo->readPointer);
	if(fill < 0) fill += (fifo->size);
	return(fill);
}

int fifo_uint8GetRemain(uint8Fifo_t* fifo){
	int fill = fifo_uint8GetFill(fifo);
	return((fifo->size) - fill - 1);
}

int fifo_uint8Clear(uint8Fifo_t* fifo){
	fifo->readPointer = fifo->writePointer;
	return(0);
}
