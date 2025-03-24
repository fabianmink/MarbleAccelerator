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


#include "interpolation.h"
#include <math.h>


#define _BV(bit) (1 << (bit))
#define bit_is_set(var, bit) (var & _BV(bit))

//Lookup-Table with
// -linear interpolation within given data points
// -extrapolation above last data point via "mode", bit0: 0=linear, 1=const
// -extrapolation below first data point via "mode", bit1 bit2:
//    * as above: 0,0
//    * symmetrical  1,0
//    * anti-symmetrical 1,1
static float lookup_lin1_mode(int nb, unsigned char mode, float b[], float data[], float x){

	int i, idx_l=0, idx_r, as=0;
    float db, ddata, ddatadb, intp;

    if(nb == 1) return(data[0]);

    if( bit_is_set(mode, 1) ){
        if(x < b[0]){
            x = -x + 2.0f*b[0];
            if( bit_is_set(mode, 2) ) as = 1;
        }
    }

	for(i=0; i<nb; i++){
		if(x >= b[i]) idx_l++;
	}

    idx_l = idx_l - 1;

    //Funktionalitaet: idx_l auf 0...nb-2 begrenzen!
    if(idx_l < 0){ //Ist Wert unterhalb der Stuetzpunkte?
        if( bit_is_set(mode, 0) ) {
            if ( as ) return(-data[0]);
            return(data[0]);
        }
        idx_l = 0;
    }
    else {
        if(idx_l >= (nb-1)){ //Ist Wert oberhalb der Stuetzpunkte
            if( bit_is_set(mode, 0) ){
               if ( as ) return(-data[nb-1]);
               return(data[nb-1]);
            }
            idx_l = nb-2;
        }
        else{ //Ansonsten ist Wert innerhalb der Stuetzpunkte
        }
    }

    //Rechter index ist einfach eins weiter als linker
    idx_r = idx_l + 1;

    db    = b[idx_r]-b[idx_l];
    ddata = data[idx_r]-data[idx_l];

    ddatadb = ddata/db;

    intp = data[idx_l] + ddatadb*( x - b[idx_l]);

    if(as) return(-intp);
    return(intp);
}



//2D Lookup-Table with
// -bilinear interpolation within given data points
// -extrapolation in x-direction:
//   -extrapolation above last data point via "mode", bit0: 0=linear, 1=const
//   -extrapolation below first data point via "mode", bit1 bit2:
//     * as above: 0,0
//     * symmetrical  1,0
//     * anti-symmetrical 1,1
//  -extrapolation in y-direction: Same as x, but with bits 4,5,6
float lookup_lin2_mode(int nbx, int nby, unsigned char mode, float bx[], float by[], float data[], float x, float y){
    int i, idx_l=0, idx_r, as=0;
    float db, ddata, ddatadb, intp, intpx0, intpx1;

    unsigned char modex;
    unsigned char modey;

    modex = mode & 0x0F;

    if(nby == 1) return(  lookup_lin1_mode(nbx, modex, bx, data, x)   );

    modey = (mode & 0xF0) >> 4;

    if( bit_is_set(modey, 1) ){
        if(y < by[0]){
            y = -y + 2.0f*by[0];
            if( bit_is_set(modey, 2) ) as = 1;
        }
    }

    for(i=0; i<nby; i++){
		if(y >= by[i]) idx_l++;
	}

    idx_l = idx_l - 1;

    //Funktionalitaet: idx_l auf 0...nby-2 begrenzen!
    if(idx_l < 0){ //Ist Wert unterhalb der Stuetzpunkte?
        if( bit_is_set(modey, 0) ) {
            if(as) return( -lookup_lin1_mode(nbx, modex, bx, data, x) );
            return( lookup_lin1_mode(nbx, modex, bx, data, x) );
        }
        idx_l = 0;
    }
    else {
        if(idx_l >= (nby-1)){ //Ist Wert oberhalb der Stuetzpunkte
            if( bit_is_set(modey, 0) ){
               if(as) return( -lookup_lin1_mode(nbx, modex, bx, &(data[nbx*(nby-1)]), x) );
               return( lookup_lin1_mode(nbx, modex, bx, &(data[nbx*(nby-1)]), x) );
            }
            idx_l = nby-2;
        }
        else{ //Ansonsten ist Wert innerhalb der Stuetzpunkte
        }
    }

    //Rechter index ist einfach eins weiter als linker
    idx_r = idx_l + 1;

    db    = by[idx_r]-by[idx_l];
    intpx0 = lookup_lin1_mode(nbx, modex, bx, &(data[nbx*idx_l]), x);
    intpx1 = lookup_lin1_mode(nbx, modex, bx, &(data[nbx*idx_r]), x);
    ddata = intpx1-intpx0;

    ddatadb = ddata/db;

    intp = intpx0 + ddatadb*( y - by[idx_l]);

    if(as) return(-intp);
    return(intp);
}


float interpolation_lin1(lin1_data_t* data, float x){
	return(lookup_lin1_mode(data->nb, data->mode, data->b, data->val, x));
}


float interpolation_lin2(lin2_data_t* data, float x, float y){
	return(lookup_lin2_mode(data->nbx, data->nbx, data->mode, data->bx, data->by, data->val, x, y));
}



