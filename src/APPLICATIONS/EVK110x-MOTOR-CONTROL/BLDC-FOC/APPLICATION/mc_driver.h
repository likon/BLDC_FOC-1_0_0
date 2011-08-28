/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Motor Control file.
 *
 * This file contains the definition for Motor Control.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:            AVR32 - Using the AVR32 FAT
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr32@atmel.com
 *                       Adapted by Marcus Jansson for Eberharter Elektronik, GmbH
 *
 ***************************************************************************/

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


#ifndef _MC_DRIVER_H_
#define _MC_DRIVER_H_
#include "compiler.h"


/*! \brief Low Level Motor Initialize
 */
extern void mc_global_init(void);
/*! \brief Low Level Start Motor
 */
extern void mc_lowlevel_start(void);
/*! \brief Low Level Stop Motor
 */
extern void mc_lowlevel_stop(void);
/*! \brief Motor Register Callback
 */
extern void mc_register_callback(void (*motor_appli_callback)(void));
/*! \brief Motor Unregister Callback
 */
extern void mc_unregister_callback();
/*! \brief Return IA current value
 */
extern unsigned long mc_get_ia(void);
/*! \brief Return IB current value
 */
extern unsigned long mc_get_ib(void);
/*! \brief Return IC current value
 */
extern unsigned long mc_get_ic(void);
/*! \brief Update Duty Cycle
 */
extern void mc_update_duty_cycle( volatile U16 mc_duty0,
                                  volatile U16 mc_duty1,
                                  volatile U16 mc_duty2);

//~ extern void mc_update_duty_cycle( volatile U16 mc_duty0,
                                  //~ volatile U16 mc_duty1,
                                  //~ volatile U16 mc_duty2,
                                  //~ volatile U16 mc_duty3,
                                  //~ volatile U16 mc_duty4,
                                  //~ volatile U16 mc_duty5);


#endif // _MC_DRIVER_H_
