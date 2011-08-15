/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Management of the USB device CDC task.
 *
 * This file manages the USB device CDC task.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
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

#include <stdio.h>
#include "usart.h"     // Shall be included before FreeRTOS header files, since 'inline' is defined to ''; leading to
                       // link errors
#include "conf_usb.h"


#if USB_DEVICE_FEATURE == ENABLED

#include "board.h"
#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "usb_drv.h"
#include "gpio.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"
#include "device_cdc_task.h"
#include "uart_usb_lib.h"
#include "frame.h"
#include "conf_foc.h"
#include "mc_control.h"
//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

static volatile U16  sof_cnt;

extern frame_message msg_idm;
extern frame_message msg_idref;
extern frame_message msg_iqm;
extern frame_message msg_iqref;
extern frame_message msg_vm;
extern frame_message msg_vref;
extern frame_message msg_speedm;
extern frame_message msg_speedest;
extern frame_message msg_tetam;
extern frame_message msg_tetaest;

extern frame_message msg_rx_vref;
extern volatile MC_BLDC_motor_t MC_BLDC_motor;

volatile unsigned char msg_rdy;
volatile unsigned char msg_sent;
//!
//! @brief This function initializes the hardware/software resources
//! required for device CDC task.
//!
void device_cdc_task_init(void)
{
  sof_cnt   = 0 ;
  uart_usb_init();
  Usb_enable_sof_interrupt();
  msg_sent = 1;
  msg_rdy  = 0; 
}


//!
//! @brief Entry point of the device CDC task management
//!
void device_cdc_task(void)
{
 if( uart_usb_tx_ready() )      // "USART"-USB free ?
 {                              
    if( (msg_rdy==1) && (msg_sent==0))  
    {
      uart_usb_sendmessage(&msg_idm,&msg_idref,&msg_iqm,&msg_iqref,&msg_vm,&msg_vref,&msg_speedm,&msg_speedest,&msg_tetam,&msg_tetaest);  
      msg_rdy = 0;
      msg_sent = 1;
    }
 }
 if (uart_usb_test_hit() )
 {      
       uart_usb_readmessage(&msg_rx_vref);      
       if ((msg_rx_vref.data[0])+(msg_rx_vref.data[1]<<8)+(msg_rx_vref.data[2]<<16)+(msg_rx_vref.data[3]<<24)!=0)
         MC_BLDC_motor.Speedref = (msg_rx_vref.data[0])+(msg_rx_vref.data[1]<<8)+(msg_rx_vref.data[2]<<16)+(msg_rx_vref.data[3]<<24);        
 }
}


//!
//! @brief usb_sof_action
//!
//! This function increments the sof_cnt counter each time
//! the USB Start-of-Frame interrupt subroutine is executed (1 ms).
//! Useful to manage time delays
//!
void usb_sof_action(void)
{
  sof_cnt++;
}

#endif  // USB_DEVICE_FEATURE == ENABLED

