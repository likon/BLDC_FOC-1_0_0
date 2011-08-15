/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief PARK for AVR32 UC3.
 *
 * AVR32 PARK module.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a TC module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (C) 2006-2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//_____  I N C L U D E S ___________________________________________________

#include <avr32/io.h>
#include "compiler.h"

extern const int coss[361];
extern const int sinn[361];

void park(int ialpha, int ibeta,unsigned short teta, int *Id, int *Iq)
{
  // Id = ialpha cos(teta)+ibeta sin(teta)
  // Iq= ibeta cos(teta)- ialpha sin( teta)

  *Id=(int)((((long long int)ialpha*(long long int)coss[teta])+((long long int)ibeta*(long long int)sinn[teta]))>>31);
  *Iq=(int)((((long long int)ibeta*(long long int)coss[teta])-((long long int)ialpha*(long long int)sinn[teta]))>>31);
}

void park_inv(int Vd, int Vq,unsigned short teta, volatile int  *valpha, volatile int *vbeta)
{
  // valpha = Vd cos(teta)-Vq sin(teta)
  // vbeta= Vq cos'teta)+ Vd sin( teta)

  *valpha=(int)((((long long int)Vd*(long long int)coss[teta])-((long long int)Vq*(long long int)sinn[teta]))>>31);
  *vbeta =(int)((((long long int)Vq*(long long int)coss[teta])+((long long int)Vd*(long long int)sinn[teta]))>>31);
}

//@}

