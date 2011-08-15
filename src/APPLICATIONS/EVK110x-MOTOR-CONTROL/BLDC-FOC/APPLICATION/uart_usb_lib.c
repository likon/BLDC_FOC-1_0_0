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


//_____  I N C L U D E S ___________________________________________________
#include "compiler.h"
#include "conf_usb.h"
#include "usb_drv.h"
#include "uart_usb_lib.h"
//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

U8    dev_rx_cnt;
U8    dev_tx_cnt;

//_____ D E C L A R A T I O N S ____________________________________________


void uart_usb_init(void)
{
  dev_rx_cnt=0 ;
  dev_tx_cnt=0 ;
}

// Functions that manage characters input through USB
//

Bool uart_usb_test_hit(void)
{
  if( dev_rx_cnt==0 )
  {
    if( Is_usb_out_received(RX_EP) )
    {
      dev_rx_cnt = Usb_byte_count(RX_EP);
      Usb_reset_endpoint_fifo_access(RX_EP);
      if( dev_rx_cnt==0 )
      {
        Usb_ack_out_received_free(RX_EP);
        return FALSE;
      }
      else
      {
        return TRUE;
      }
    }
    else
    {     
      return FALSE;
    }
  }
  else
  { 
    return TRUE;
  }
}

char uart_usb_getchar(void)
{
  register char data_rx;

  while( !uart_usb_test_hit() );

  data_rx=Usb_read_endpoint_data(RX_EP, 8);
  dev_rx_cnt--;
  if( dev_rx_cnt==0 ) Usb_ack_out_received_free(RX_EP);

  return data_rx;
}



// Functions that manage characters output through USB
//

Bool uart_usb_tx_ready(void)
{
  if( !Is_usb_write_enabled(TX_EP) )
    return FALSE;

  return TRUE;
}

int uart_usb_putchar(int data_to_send)
{
  while( !uart_usb_tx_ready() ); // Wait Endpoint ready

  if( dev_tx_cnt==0 )
  {
    Usb_reset_endpoint_fifo_access(TX_EP);
  }
  Usb_write_endpoint_data(TX_EP, 8, data_to_send);
  dev_tx_cnt++;
  if( !uart_usb_tx_ready() ) //If Endpoint full -> flush
  {
    uart_usb_flush();
  }
  return data_to_send;
}

#define RXBUF_SIZE        (5*6)
char rxbuf[RXBUF_SIZE];
void uart_usb_readmessage(frame_message *msg_rx_vref)
{

  Usb_reset_endpoint_fifo_access(RX_EP);     
  usb_read_ep_rxpacket(RX_EP, &rxbuf, RXBUF_SIZE, NULL);
  
  msg_rx_vref->data[0] = rxbuf[2];
  msg_rx_vref->data[1] = rxbuf[3];
  msg_rx_vref->data[2] = rxbuf[4];
  msg_rx_vref->data[3] = rxbuf[5];  
  
  dev_rx_cnt = 0;  
  Usb_ack_out_received_free(RX_EP);
}


#define TXBUF_SIZE        (10*6)
char txbuf[TXBUF_SIZE];
void uart_usb_sendmessage(frame_message *msgidm,
                          frame_message *msgidref,
                          frame_message *msgiqm,
                          frame_message *msgiqref,
                          frame_message *msgvm,
                          frame_message *msgvref,
                          frame_message *msg_speedm,                          
                          frame_message *msg_speedest,
                          frame_message *msg_tetam,
                          frame_message *msg_tetaest)
{

      Usb_reset_endpoint_fifo_access(TX_EP);    
      txbuf[0] = msgidm->cmd;
      txbuf[1] = msgidm->dlc;
      txbuf[2] = msgidm->data[0];
      txbuf[3] = msgidm->data[1];
      txbuf[4] = msgidm->data[2];
      txbuf[5] = msgidm->data[3];  
      txbuf[6] = msgidref->cmd;
      txbuf[7] = msgidref->dlc;
      txbuf[8] = msgidref->data[0];
      txbuf[9] = msgidref->data[1];
      txbuf[10] = msgidref->data[2];
      txbuf[11] = msgidref->data[3]; 
      txbuf[12] = msgiqm->cmd;
      txbuf[13] = msgiqm->dlc;
      txbuf[14] = msgiqm->data[0];
      txbuf[15] = msgiqm->data[1];
      txbuf[16] = msgiqm->data[2];
      txbuf[17] = msgiqm->data[3];   
      txbuf[18] = msgiqref->cmd;
      txbuf[19] = msgiqref->dlc;
      txbuf[20] = msgiqref->data[0];
      txbuf[21] = msgiqref->data[1];
      txbuf[22] = msgiqref->data[2];
      txbuf[23] = msgiqref->data[3];      
      txbuf[24] = msgvm->cmd;
      txbuf[25] = msgvm->dlc;
      txbuf[26] = msgvm->data[0];
      txbuf[27] = msgvm->data[1];
      txbuf[28] = msgvm->data[2];
      txbuf[29] = msgvm->data[3];     
      txbuf[30] = msgvref->cmd;
      txbuf[31] = msgvref->dlc;
      txbuf[32] = msgvref->data[0];
      txbuf[33] = msgvref->data[1];
      txbuf[34] = msgvref->data[2];
      txbuf[35] = msgvref->data[3];    
      txbuf[36] = msg_speedm->cmd;
      txbuf[37] = msg_speedm->dlc;
      txbuf[38] = msg_speedm->data[0];
      txbuf[39] = msg_speedm->data[1];
      txbuf[40] = msg_speedm->data[2];
      txbuf[41] = msg_speedm->data[3];       
      txbuf[42] = msg_speedest->cmd;
      txbuf[43] = msg_speedest->dlc;
      txbuf[44] = msg_speedest->data[0];
      txbuf[45] = msg_speedest->data[1];
      txbuf[46] = msg_speedest->data[2];
      txbuf[47] = msg_speedest->data[3];        
      txbuf[48] = msg_tetam->cmd;
      txbuf[49] = msg_tetam->dlc;
      txbuf[50] = msg_tetam->data[0];
      txbuf[51] = msg_tetam->data[1];
      txbuf[52] = msg_tetam->data[2];
      txbuf[53] = msg_tetam->data[3];  
      txbuf[54] = msg_tetaest->cmd;
      txbuf[55] = msg_tetaest->dlc;
      txbuf[56] = msg_tetaest->data[0];
      txbuf[57] = msg_tetaest->data[1];
      txbuf[58] = msg_tetaest->data[2];
      txbuf[59] = msg_tetaest->data[3];       
      dev_tx_cnt = TXBUF_SIZE-1;
      usb_write_ep_txpacket(TX_EP, &txbuf, TXBUF_SIZE, NULL);
      uart_usb_flush();
}


void uart_usb_flush (void)
{
  Bool zlp=FALSE;
  if( dev_tx_cnt!=0 )
  {
    if(!Is_usb_write_enabled(TX_EP))              // Endpoint full, need ZLP
       zlp=TRUE;

    Usb_ack_in_ready_send(TX_EP);

    if( zlp==TRUE )
    {
       while( !Is_usb_write_enabled(TX_EP) );     // Wait Endpoint ready...
       Usb_ack_in_ready_send(TX_EP);              // ...and Send ZLP
    }
    dev_tx_cnt = 0;
  }
}
