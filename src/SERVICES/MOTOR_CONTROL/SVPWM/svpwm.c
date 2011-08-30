/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief SVPWM Library for AVR32 UC3.
 *
 * SVPWM Library module.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a TC module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. ATMEL grants developer a non-exclusive, limited license to use the Software
 * as a development platform solely in connection with an Atmel AVR product
 * ("Atmel Product").
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */

//_____  I N C L U D E S ___________________________________________________

#include <avr32/io.h>
#include "compiler.h"
#include "svpwm.h"
//_____ D E F I N I T I O N S ______________________________________________


//------------------------------------------------------------------------------
/*! \name Compute SVPWM
 */
//! @{
// Valpha et Vbeta are divided by 2E
void svpwm(volatile svpwm_options_t *svpwm_options)
{
   unsigned int tempsr0, tempsr2, tempsr4;
   unsigned int delta_i, delta_2i, delta_z;
   int Vb_M_Va, Vb_P_Va, Va_P_Vb, Va_M_Vb;

   //******************************************************************
   // Vbeta-sqrt(3)*Valpha
   Vb_M_Va = svpwm_options->Vbeta - (svpwm_options->Valpha <<1) + (svpwm_options->Valpha >>2) + (svpwm_options->Valpha >>6);
   // Vbeta+sqrt(3)*Valpha
   Vb_P_Va = (svpwm_options->Vbeta <<1) - Vb_M_Va;
   if (svpwm_options->Vbeta >0)
   {
     if (svpwm_options->Valpha >0)
     {
       if (Vb_M_Va <0) // SECTOR =i =1
       { // delta_2i= 2*sqrt(2)* Vbeta
          svpwm_options->current_to_be_measured=BC;
          delta_2i = (svpwm_options->Vbeta<<1) + (svpwm_options->Vbeta>>1) + (svpwm_options->Vbeta>>2) + (svpwm_options->Vbeta>>4)+ (svpwm_options->Vbeta>>6);

         // delta_i = 2*sqrt(1.5)*Valpha - 0.5*delta_2i
          delta_i = (svpwm_options->Valpha<<1) + (svpwm_options->Valpha>>1) - (svpwm_options->Valpha>>4) - (delta_2i>>1);

         // delta_z = 0.5*( 1 - delta_2i - delta_i )
          delta_z = (Un_1 - delta_2i - delta_i)>>1;

          tempsr0 = delta_z + delta_2i + delta_i;
          tempsr2 = delta_2i + delta_z;
          tempsr4 = delta_z;
       }

       else // SECTOR = i = 2
       {
         svpwm_options->current_to_be_measured=AC;
         //Valpha-sqrt(3)*Vbeta
         Va_P_Vb = svpwm_options->Valpha + (svpwm_options->Vbeta<<1) - (svpwm_options->Vbeta>>2) - (svpwm_options->Vbeta>>6);
         // delta_2i = sqrt(2)* Vb_M_Va
         delta_2i = (Vb_M_Va<<1) - (Vb_M_Va>>1) -(Vb_M_Va>>4) -(Vb_M_Va>>5);
         // delta_i = sqrt(1.5)*(Valpha+sqrt(3)*Vbeta) - 0.5*delta_2i
         delta_i  = Va_P_Vb + (Va_P_Vb>>2) - (Va_P_Vb>>5) - (delta_2i>>1);
         delta_z  = (Un_1 - delta_2i - delta_i)>>1;
         tempsr0  = delta_i + delta_z;
         tempsr2  = delta_i + delta_2i + delta_z;
         tempsr4  = delta_z;
       }
     }
     else
       if (Vb_P_Va <0)  // SECTOR = i = 3
       {
         svpwm_options->current_to_be_measured=CA;
         // delta_2i = -sqrt(2)*Vb_P_Va
         delta_2i = -(Vb_P_Va<<1) + (Vb_P_Va>>1) + (Vb_P_Va>>4) + (Vb_P_Va>>5);
         // Valpha-sqrt(3)*Vbeta
         Va_M_Vb = svpwm_options->Valpha - (svpwm_options->Vbeta<<1) + (svpwm_options->Vbeta>>2) + (svpwm_options->Vbeta>>6);
         // delta_i = -sqrt(1.5)*(Va_M_Vb)- 0.5*delta_2i
         delta_i = - Va_M_Vb - (Va_M_Vb>>2) + (Va_M_Vb>>5) - (delta_2i>>1);
         delta_z = (Un_1 - delta_2i - delta_i)>>1;
         tempsr0 = delta_z;
         tempsr2 = delta_i + delta_2i+ delta_z;
         tempsr4 = delta_2i + delta_z;
       }
     else  // SECTOR = i = 2
     {

         svpwm_options->current_to_be_measured=AC;
         //Valpha+sqrt(3)*Vbeta
         Va_P_Vb = svpwm_options->Valpha + (svpwm_options->Vbeta<<1) - (svpwm_options->Vbeta>>2) - (svpwm_options->Vbeta>>6);
         // delta_2i = sqrt(2)* Vb_M_Va
         delta_2i = (Vb_M_Va<<1) - (Vb_M_Va>>1) -(Vb_M_Va>>4) -(Vb_M_Va>>5);
         // delta_i = sqrt(1.5)*(Valpha+sqrt(3)*Vbeta) - 0.5*delta_2i
         delta_i  = Va_P_Vb + (Va_P_Vb>>2) - (Va_P_Vb>>5) - (delta_2i>>1);
         delta_z  = (Un_1 - delta_2i - delta_i)>>1;
         tempsr0  = delta_i + delta_z;
         tempsr2  = delta_i + delta_2i + delta_z;
         tempsr4  = delta_z;
     }
   }
   else
   {
      if (svpwm_options->Valpha >0)
      {
        if (Vb_P_Va <0)// SECTOR = i = 5
        {
          svpwm_options->current_to_be_measured=AB;
          // delta_2i = - sqrt(2)*( Vbeta - sqrt(3)*Valpha)
          delta_2i = -(Vb_M_Va<<1) + (Vb_M_Va>>1) + (Vb_M_Va>>4) + (Vb_M_Va>>5);
          //Valpha+sqrt(3)*Vbeta
          Va_P_Vb = svpwm_options->Valpha + (svpwm_options->Vbeta<<1) - (svpwm_options->Vbeta>>2) - (svpwm_options->Vbeta>>6);
          // delta_i = - sqrt(1.5)*(Valpha+sqrt(3)*Vbeta) - 0.5* delta_2i
          delta_i  = - Va_P_Vb - (Va_P_Vb>>2) + (Va_P_Vb>>5) - (delta_2i>>1);
          delta_z  = (Un_1 - delta_2i - delta_i)>>1;
          tempsr0  = delta_2i + delta_z;
          tempsr2  = delta_z;
          tempsr4  = delta_z + delta_2i + delta_i;
        }
        else   // SECTOR = i = 6
        {
          svpwm_options->current_to_be_measured=CB;
          // delta_2i =  sqrt(2)*( Vbeta + sqrt(3)*Valpha)
          delta_2i = (Vb_P_Va<<1) - (Vb_P_Va>>1) - (Vb_P_Va>>4) - (Vb_P_Va>>5);
          // Valpha-sqrt(3)*Vbeta
          Va_M_Vb = svpwm_options->Valpha - (svpwm_options->Vbeta<<1) + (svpwm_options->Vbeta>>2) + (svpwm_options->Vbeta>>6);
          // delta_i =  sqrt(1.5)*(Valpha-sqrt(3)*Vbeta) - 0.5* delta_2i
          delta_i = Va_M_Vb + (Va_M_Vb>>2) - (Va_M_Vb>>5) - (delta_2i>>1);
          delta_z  = (Un_1 - delta_2i - delta_i)>>1;
          tempsr0  = delta_2i + delta_z + delta_i;
          tempsr2  = delta_z;
          tempsr4  = delta_z + delta_i;
        }
      }
      else
        if (Vb_M_Va <0)// SECTOR = i = 5
        {
          svpwm_options->current_to_be_measured=AB;
          // delta_2i = - sqrt(2)*( Vbeta - sqrt(3)*Valpha)
          delta_2i = -(Vb_M_Va<<1) + (Vb_M_Va>>1) + (Vb_M_Va>>4) + (Vb_M_Va>>5);
          //Valpha+sqrt(3)*Vbeta
          Va_P_Vb = svpwm_options->Valpha + (svpwm_options->Vbeta<<1) - (svpwm_options->Vbeta>>2) - (svpwm_options->Vbeta>>6);
          // delta_i = - sqrt(1.5)*(Valpha+sqrt(3)*Vbeta) - 0.5* delta_2i
          delta_i  = - Va_P_Vb - (Va_P_Vb>>2) + (Va_P_Vb>>5) - (delta_2i>>1);
          delta_z  = (Un_1 - delta_2i - delta_i)>>1;
          tempsr0  = delta_2i + delta_z;
          tempsr2  = delta_z;
          tempsr4  = delta_z + delta_2i + delta_i;
        }
      else // SECTOR = i = 4
      {
          svpwm_options->current_to_be_measured=BA;
         // delta_2i= -2*sqrt(2)* Vbeta
          delta_2i = -(svpwm_options->Vbeta<<1)- (svpwm_options->Vbeta>>1) - (svpwm_options->Vbeta>>2) - (svpwm_options->Vbeta>>4)- (svpwm_options->Vbeta>>6);
         // delta_i = -2*sqrt(1.5)*Valpha - 0.5*delta_2i
          delta_i = -(svpwm_options->Valpha<<1) - (svpwm_options->Valpha>>1) + (svpwm_options->Valpha>>4) - (delta_2i>>1);
          delta_z = (Un_1 - delta_2i - delta_i)>>1;
          tempsr0  = delta_z;
          tempsr2  = delta_z + delta_i;
          tempsr4  = delta_z + delta_2i + delta_i;
      }
   }

  svpwm_options->duty0 = (tempsr0>>20) - (tempsr0>>22) - (tempsr0>>23) - (tempsr0>>25) - (tempsr0>>26);//-10;
  svpwm_options->duty1 = (tempsr2>>20) - (tempsr2>>22) - (tempsr2>>23) - (tempsr2>>25) - (tempsr2>>26);//-10;
  svpwm_options->duty2 = (tempsr4>>20) - (tempsr4>>22) - (tempsr4>>23) - (tempsr4>>25) - (tempsr4>>26);//-10;

  //~ printf("d0 = 0x%x\n\r", svpwm_options->duty0);
  //~ printf("d1 = 0x%x\n\r", svpwm_options->duty1);
  //~ printf("d2 = 0x%x\n\r", svpwm_options->duty2);
}
//@}

