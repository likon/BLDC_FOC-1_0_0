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
//_____  I N C L U D E S ___________________________________________________
#include "compiler.h"
#include "mc_control.h"
#include "mc_driver.h"
#include "frame.h"
#include "foc.h"
#include "conf_foc.h"
//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

//!< Current Motor State value.
volatile MC_BLDC_motor_t MC_BLDC_motor;
//!< Message OUTPUT definition for Idm current.
frame_message msg_idm;
//!< Message OUTPUT definition for Idref current.
frame_message msg_idref;
//!< Message OUTPUT definition for Iqm current.
frame_message msg_iqm;
//!< Message OUTPUT definition for Iqref current.
frame_message msg_iqref;
//!< Message OUTPUT definition for Electrical Speed value.
frame_message msg_vm;
//!< Message OUTPUT definition for Electrical Speed Reference value.
frame_message msg_vref;
//!< Message OUTPUT definition for Electrical Speed value.
frame_message msg_speedm;
//!< Message OUTPUT definition for Estimated Electrical Speed value.
frame_message msg_speedest;
//!< Message OUTPUT definition for Electrical Position value.
frame_message msg_tetam;
//!< Message OUTPUT definition for Estimated Position value.
frame_message msg_tetaest;
//!< Message INPUT definition for reference Speed value.
frame_message msg_rx_vref;

//!< Tick reference state
extern volatile unsigned short tick;
//!< Flag for Message INPUT Synchronisation
extern volatile unsigned char msg_rdy;
//!< Flag for Message OUTPUT Synchronisation
extern volatile unsigned char msg_sent;

void mc_control_task_init()
{
    // Initialize messages
    msg_idm.dlc = 4; msg_idm.cmd = FRAME_SENDIDM_CMD;
    msg_idref.dlc = 4; msg_idref.cmd = FRAME_SENDIDREF_CMD;
    msg_iqm.dlc = 4; msg_iqm.cmd = FRAME_SENDIQM_CMD;
    msg_iqref.dlc = 4; msg_iqref.cmd = FRAME_SENDIQREF_CMD;
    msg_vm.dlc = 4; msg_vm.cmd = FRAME_SENDSPEEDM_CMD;
    msg_vref.dlc = 4; msg_vref.cmd = FRAME_SENDSPEEDREF_CMD;
    msg_speedm.dlc = 4; msg_speedm.cmd = FRAME_SENDSPEEDMES_CMD;
    msg_speedest.dlc = 4; msg_speedest.cmd = FRAME_SENDSPEEDEST_CMD;
    msg_tetam.dlc = 4; msg_tetam.cmd = FRAME_SENDTETAM_CMD;
    msg_tetaest.dlc = 4; msg_tetaest.cmd = FRAME_SENDTETAEST_CMD;
    msg_rx_vref.dlc = 4;msg_rx_vref.cmd = FRAME_GETVREF;

    MC_BLDC_motor.mc_state = INIT;
}

void mc_control_task()
{
   switch(MC_BLDC_motor.mc_state)
   {
      case INIT:
        mc_global_init();
        MC_BLDC_motor.mc_state = START;
        break;
      case STOP:
        mc_unregister_callback();
        mc_lowlevel_stop();
        break;
      case START:
        mc_lowlevel_start();
        mc_register_callback(FOC_state_machine);
        FOC_set_state_machine(FOC_state_ramp_up_init);
        MC_BLDC_motor.mc_state = RUN;
        break;
      case RUN:
		//Add code here. Or do nothing, except wait for next PWM pulse.
        break;
   }
}

void mc_init()
{
   MC_BLDC_motor.mc_state = INIT;
}

void mc_start()
{
   MC_BLDC_motor.mc_state = START;
}

void mc_stop()
{
  MC_BLDC_motor.mc_state = STOP;
}

void mc_forward()
{
  MC_BLDC_motor.mc_state = STOP;
  mc_set_motor_direction(MC_CW);
}

void mc_backward()
{
  MC_BLDC_motor.mc_state = STOP;
  mc_set_motor_direction(MC_CCW);
}
//------------------------------------------------------------------------------
/*! \name Set Motor Direction
 */
//! @{
void mc_set_motor_direction(mc_motor_direction_t direction)
{
  MC_BLDC_motor.mc_motor_direction = direction;
}
//------------------------------------------------------------------------------
/*! \name Get Motor Direction
 */
//! @{
mc_motor_direction_t mc_get_motor_direction(void)
{
  return MC_BLDC_motor.mc_motor_direction;
}