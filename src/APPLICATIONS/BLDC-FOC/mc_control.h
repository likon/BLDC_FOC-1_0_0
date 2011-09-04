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

#ifndef _MC_CONTROL_H_
#define _MC_CONTROL_H_

#include "compiler.h"

//! Motor State Value.
typedef enum {
  INIT = 0,
  STOP=1,
  START=2,
  RUN=3
} mc_motor_state_t;

//! Motor Direction value.
typedef enum {
  MC_CW=0,
  MC_CCW=1
} mc_motor_direction_t;

typedef struct
{
  //! Current Motor State
  mc_motor_state_t mc_state;
  
  //! Current Motor Direction
  mc_motor_direction_t mc_motor_direction;
  
  //! Id current value measured.
  int Idm;
  //! Id current value as reference.
  int Idref;
  //! Iq current value measured.
  int Iqm;
  //! Iq current value as reference.
  int Iqref;
  //! Speed current value measured.
  unsigned short Speedm;
  //! Speed current value as reference.
  int  Speedref;
  //! Speed current value estimated.
  unsigned short SpeedEst;
  //! Teta current value measured.
  unsigned short Tetam;
  //! Teta current value as reference.
  unsigned short Tetaref;
  //! Teta current value estimated.
  unsigned short TetaEst;
}MC_BLDC_motor_t;

/*! \brief Control Task Init
 */
extern void mc_control_task_init();
/*! \brief Control Task loop
 */
extern void mc_control_task();
/*! \brief High Level start motor control sequence
 */
extern void mc_start();
/*! \brief High Level run motor control
 */
extern void mc_run();
/*! \brief High Level stop motor control
 */
extern void mc_stop();
/*! \brief High Level change direction motor control in FW
 */
extern void mc_forward();
/*! \brief High Level change direction motor control in BW
 */
extern void mc_backward();
/*! \brief High Level Set direction motor control 
 */
extern void mc_set_motor_direction(mc_motor_direction_t direction);
/*! \brief High Level get direction motor control 
 */
extern U8 mc_get_direction(void);
#endif
