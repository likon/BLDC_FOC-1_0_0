/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief This file controls the UART USB functions.
 *
 * These functions allow to use en USB endpoint as we would do using an UART.
 * This is particurly well suited for USB CDC class.
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


#ifndef _UART_USB_LIB_H
#define _UART_USB_LIB_H

#include "frame.h"
//_____ I N C L U D E S ____________________________________________________

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

//! @brief This function initializes the UART over USB emulation driver.
//!
void uart_usb_init(void);

//! @brief This function checks if a character has been received on the USB bus.
//!
//! @return Bool (true if a byte is ready to be read)
//!
Bool uart_usb_test_hit(void);

//! @brief This function reads one byte from the USB bus
//!
//! If one byte is present in the USB fifo, this byte is returned. If no data
//! is present in the USB fifo, this function waits for USB data.
//!
//! @return U8 byte received
//!
char uart_usb_getchar(void);

//! @brief This function checks if the USB emission buffer is ready to accept at
//! at least 1 byte
//!
//! @return Boolean. TRUE if the firmware can write a new byte to transmit.
//!
Bool uart_usb_tx_ready(void);

//! @brief This function fills the USB transmit buffer with the new data. This buffer
//! is sent if complete. To flush this buffer before waiting full, launch
//! the uart_usb_flush() function.
//!
//! @param data_to_send
//!
//! @return
//!
int  uart_usb_putchar(int data_to_send);

//! @brief This function sends the data stored in the USB transmit buffer.
//! This function does nothing if there is no data in the buffer.
//!
void uart_usb_flush (void);


void uart_usb_sendmessage(frame_message *msgidm,
                          frame_message *msgidref,
                          frame_message *msgiqm,
                          frame_message *msgiqref,
                          frame_message *msgvm,
                          frame_message *msgvref,
                          frame_message *msg_speedm,                          
                          frame_message *msg_speedest,
                          frame_message *msg_tetam,
                          frame_message *msg_tetaest);

void uart_usb_readmessage(frame_message *msg_rx_vref);
#endif  // _UART_USB_LIB_H
