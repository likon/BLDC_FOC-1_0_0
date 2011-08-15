/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Processing of Field Oriented Control Process.
 *
 * This file contains the Execution of Field Oriented Control Process.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
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


//_____ I N C L U D E S ____________________________________________________

#include "foc.h"
#include "svpwm.h"
#include "mc_control.h"
#include "hall_estimator.h"
#include "mc_driver.h"
#include "park.h"
#include "concordia.h"
#include "ip_reg.h"
#include "util.h"
#include "CONF/conf_foc.h"
#include "CONF/cos_sin.h"
#include "CONF/mc300.h"
//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

int Vd=0;
int Vq=0;
int ialpha=0;
int ibeta=0;
int Vdref=0;
int Vqref=0;

//_____ P R I V A T E   D E C L A R A T I O N S ____________________________
//!< Current svpwm value
volatile svpwm_options_t svpwm_options;
//!< Tick reference for Speed Regulation
volatile unsigned short FOC_tick_speed = 0; 
//!< Current IA value
int ia = 0;
//!< Current IB value
int ib = 0;
//!< Current IC value
int ic = 0;
//!< Id structure for regulation.
volatile IP_REG_variables_t FOC_Id_reg;
//!< Iq structure for regulation.
volatile IP_REG_variables_t FOC_Iq_reg;
//!< Speed structure for regulation.
volatile IP_REG_variables_t FOC_Speed_reg;

volatile int FOC_rampup_step_alignement_counter = 0;
volatile int FOC_rampup_step_openloop_counter   = 0;
volatile int FOC_rampup_step_fieldreg_counter   = 0;
volatile int FOC_rampup_step_torquereg_counter  = 0;

volatile FOC_STATE_t FOC_state;
volatile FOC_RAMPUP_STATE_t FOC_rampup_step;

extern const int coss[361];
extern const int sinn[361];
extern volatile MC_BLDC_motor_t MC_BLDC_motor;

//!< Local Read Current Definition
static void FOC_read_current(void);
//!< Local Vd Vq computation function
static void FOC_computeVdVq(void);
//!< Local Update Position and Speed
static void FOC_update_teta_speed(void);
//!< Local Clarke Transformation
static void FOC_compute_clarke(void);
//!< Local Park Transformation
static void FOC_compute_park(void);
//!< Local Inverse Park Transformation
static void FOC_compute_inv_park(void);
//!< Local Torque Regulation
static void FOC_regulate_torque(void);
//!< Local Field Regulation
static void FOC_regulate_field(void);
//!< Local SVPWM computation function
static void FOC_compute_svpwm(void);
//!< Local Update of duty cycle
static void FOC_update_duty(void);
//!< Local Start motor sequence
static unsigned char FOC_start_motor(void);

void FOC_set_state_machine(FOC_STATE_t FOC_state_p) { FOC_state = FOC_state_p;}

//! @brief This function executes FOC treatment 
//! This function is called at every tick reference.
//!
void FOC_state_machine(void)
{
  switch(FOC_state)
  {
    case FOC_state_ramp_up_init:
        FOC_Id_reg.Ki = Ki_id;
        FOC_Id_reg.Kp = Kp_id;
        FOC_Iq_reg.Ki = Ki_iq;
        FOC_Iq_reg.Kp = Kp_iq;
        FOC_Speed_reg.Ki = Ki_speed;
        FOC_Speed_reg.Kp = Kp_speed;
        FOC_rampup_step = FOC_rampup_step_alignement_init;
        FOC_state = FOC_state_ramp_up;
      break;
    case FOC_state_ramp_up:
      if (FOC_start_motor()==1)
        FOC_state = FOC_state_regulation;
      break;
    case FOC_state_regulation:
      // Current Measurement
      FOC_read_current();
      // Update teta and speed values
      FOC_update_teta_speed();
      // Compute Clarke Transformation
      FOC_compute_clarke();
      // Compute Park Transformation
      FOC_compute_park();      
      // In case of speed regulation: regulate torque
      if (FOC_tick_speed)
      {
        // Regulate Torque value
        FOC_regulate_torque();
      }
      // Regulate Field value
      FOC_regulate_field();
      // Compute Inverse Park Transformation
      FOC_compute_inv_park();
      // Compute Space Vector Modulation Transformation
      FOC_compute_svpwm();
      // Update Duty Cycle
      FOC_update_duty();
      break;
  }
}

//! @brief This function executes Current Reading 
//! Theory : 
//!           ia+ib+ic=0 
//!                       or
//!           adc_value_ia + adc_value_ib + adc_value_ic = zero_adc;
//!
static void FOC_read_current(void)
{
  volatile unsigned short adc_value_ia;
  volatile unsigned short adc_value_ib;
  volatile unsigned short adc_value_ic;
  
  if(svpwm_options.current_to_be_measured == AB)
  {
    adc_value_ia = mc_get_ia();
    adc_value_ib = mc_get_ib();
    
    adc_value_ic =zero_adc-adc_value_ia-adc_value_ib;
    svpwm_options.current_to_be_measured == NONE;
  }
  else 
      if(svpwm_options.current_to_be_measured == AC)
      {
        adc_value_ia = mc_get_ia();
        adc_value_ic = mc_get_ic();
        adc_value_ib =zero_adc-adc_value_ia-adc_value_ic;
        svpwm_options.current_to_be_measured == NONE;
      }
      else 
             if (svpwm_options.current_to_be_measured == BA)
             {
                adc_value_ib = mc_get_ib();
                adc_value_ia = mc_get_ia();
                adc_value_ic =zero_adc-adc_value_ia-adc_value_ib;
                svpwm_options.current_to_be_measured == NONE;
             }
             else 
                   if (svpwm_options.current_to_be_measured == BC)
                   {
                      adc_value_ib = mc_get_ib();
                      adc_value_ic = mc_get_ic();
                      adc_value_ia =zero_adc-adc_value_ib-adc_value_ic;
                      svpwm_options.current_to_be_measured == NONE;
                   }
                   else 
                         if (svpwm_options.current_to_be_measured == CA)
                         {
                            adc_value_ic = mc_get_ic();
                            adc_value_ia = mc_get_ia();
                            adc_value_ib =zero_adc-adc_value_ia-adc_value_ic;
                            svpwm_options.current_to_be_measured == NONE;
                         }
                          else 
                                if (svpwm_options.current_to_be_measured == CB)
                                {
                                    adc_value_ic = mc_get_ic();
                                    adc_value_ib = mc_get_ib();
                                    adc_value_ia =zero_adc-adc_value_ic-adc_value_ib;
                                    svpwm_options.current_to_be_measured == NONE;
                                }
   // Rescale Current measures.
   ia= (adc_value_ia-offset)*echelle_adc;
   ib= (adc_value_ib-offset)*echelle_adc;
   ic= (adc_value_ic-offset)*echelle_adc;

}
static void FOC_computeVdVq(void)
{
  int prev_Vd;
  int prev_Vq;
  
  // Vd= Vdref-Speed.Lc.Iq
  // Vq= Vqref+omega.Lc.Id+(Speed/p).kcn
  Vd=Vdref- (int)((((long long int)Lc* (long long int)MC_BLDC_motor.Iqm)>>31)*MC_BLDC_motor.Speedm);
  Vq= Vqref+ (int)((((long long int)Lc* (long long int)MC_BLDC_motor.Idm)>>31)*MC_BLDC_motor.Speedm)+Kcn*(MC_BLDC_motor.Speedm/P);

  prev_Vd = Vd;
  prev_Vq = Vq;
  
  // if abs(Vd)>1/sqrt(8) (V.fixe) so Vd=1/sqrt(8)
  if (Vd >rayon_limitation)
  {
     Vd=rayon_limitation;
     Vq=0;
  }
  else if(Vd <-rayon_limitation)
    {
     Vd=-rayon_limitation;
     Vq=0;
    }
 else 
 {
   int Vdcarre;
   int test;
   Vdcarre=(int) (((long long int)Vd * (long long int)Vd)>>31);
   //test = 268435456 -Vqcarre;
    test=rayon_carre_limitation -(int) (((long long int)Vq*(long long int)Vq)>>31);
   if (Vdcarre>test)  // if vd^2+vq^2 >1/8
   {
    if  (Vq>0)
    {
      //*Vq=sqrt( 268435456- *Vd^2 );
      //test=268435456- Vdcarre;
      test=rayon_carre_limitation- Vdcarre;
      Vq=sqrtt(test,erreur_max);
    }
    else //*Vq= -sqrt( 268435456- *Vd^2 ); 
    {
      test=rayon_carre_limitation- Vdcarre;
      Vq=-sqrtt(test,erreur_max);
    }
   }
 }
  FOC_Id_reg.IP_REG_discharge = prev_Vd - Vd;
  FOC_Iq_reg.IP_REG_discharge = prev_Vq - Vq; 
}

static void FOC_update_teta_speed(void)
{
     hall_estimator_update_teta_and_speed(&(MC_BLDC_motor.Tetam), &(MC_BLDC_motor.Speedm));
}
static void FOC_compute_clarke(void)
{
     concordia(ia,ib,ic,&ialpha,&ibeta);
}
static void FOC_compute_park(void)
{
    park(ialpha,ibeta,MC_BLDC_motor.Tetam,&(MC_BLDC_motor.Idm),&(MC_BLDC_motor.Iqm));
}

static void FOC_regulate_torque(void)
{
    // REgulate Speed
    FOC_Speed_reg.IP_REG_mes = (MC_BLDC_motor.Speedm)*(transf_v);
    FOC_Speed_reg.IP_REG_ref = MC_BLDC_motor.Speedref;
    IP_REG_compute(&FOC_Speed_reg);
    MC_BLDC_motor.Iqref = FOC_Speed_reg.IP_REG_output*104; //(2500/24)
    FOC_Iq_reg.IP_REG_lasterror = MC_BLDC_motor.Iqref - MC_BLDC_motor.Iqm;
}
static void FOC_regulate_field(void)
{
    // REgulate Id
    FOC_Id_reg.IP_REG_mes = MC_BLDC_motor.Idm;
    FOC_Id_reg.IP_REG_ref = MC_BLDC_motor.Idref;
    IP_REG_compute(&FOC_Id_reg);
    Vdref = FOC_Id_reg.IP_REG_output;
    // REgulate Iq
    FOC_Iq_reg.IP_REG_mes = MC_BLDC_motor.Iqm;
    FOC_Iq_reg.IP_REG_ref = MC_BLDC_motor.Iqref;
    IP_REG_compute(&FOC_Iq_reg);
    Vqref = FOC_Iq_reg.IP_REG_output;
}
static void FOC_compute_inv_park(void)
{
    // Compute new Vd and Vq values
    FOC_computeVdVq();
    park_inv(Vd,Vq,MC_BLDC_motor.Tetam,&(svpwm_options.Valpha),&(svpwm_options.Vbeta));

}

static void FOC_compute_svpwm(void)
{
     svpwm(&svpwm_options);
}
static void FOC_update_duty(void)
{
  if ( MC_BLDC_motor.mc_motor_direction == MC_CW)
  {
      mc_update_duty_cycle( svpwm_options.duty0,
                            svpwm_options.duty1,
                            svpwm_options.duty2,
                            svpwm_options.duty3,
                            svpwm_options.duty4,
                            svpwm_options.duty5);
   }
   else
   {
      mc_update_duty_cycle( svpwm_options.duty2,
                            svpwm_options.duty3,
                            svpwm_options.duty0,
                            svpwm_options.duty1,
                            svpwm_options.duty4,
                            svpwm_options.duty5);
   }
}
//------------------------------------------------------------------------------
/*! \name Start Motor sequence
 */
//! @{
static unsigned char FOC_start_motor(void)
{
  switch(FOC_rampup_step)
  {
    case FOC_rampup_step_alignement_init:
        FOC_rampup_step_alignement_counter = 0;
        FOC_rampup_step_openloop_counter   = 0;
        FOC_rampup_step_fieldreg_counter   = 0;
        FOC_rampup_step_torquereg_counter  = 0;
        MC_BLDC_motor.Idref = 0;
        MC_BLDC_motor.Iqref = IQREF_RAMPUP;
        Vqref=(int)((R*(long long int)MC_BLDC_motor.Iqref)>>31)+(187904750*2);        
        park_inv(Vdref,Vqref,2,&(svpwm_options.Valpha),&(svpwm_options.Vbeta));
        FOC_compute_svpwm();
        FOC_update_duty();
        FOC_rampup_step = FOC_rampup_step_alignement;
      break;

    case FOC_rampup_step_alignement:
      FOC_rampup_step_alignement_counter++;
      if (FOC_rampup_step_alignement_counter == 5000) { 
          hall_estimator_init();
          FOC_rampup_step = FOC_rampup_step_openloop;
      }
      break;

    case FOC_rampup_step_openloop:
      FOC_rampup_step_openloop_counter++;
      FOC_update_teta_speed();
      MC_BLDC_motor.Tetam=(unsigned short)(FOC_rampup_step_openloop_counter-360*(int)(FOC_rampup_step_openloop_counter/360));
      park_inv(Vdref,Vqref,MC_BLDC_motor.Tetam,&(svpwm_options.Valpha),&(svpwm_options.Vbeta));
      FOC_compute_svpwm();
      FOC_update_duty();
      if (FOC_rampup_step_openloop_counter == 6000) {
          hall_estimator_init_teta(MC_BLDC_motor.Tetam);
          FOC_rampup_step_fieldreg_counter = 5000;
          FOC_Id_reg.IP_REG_discharge = 0;
          FOC_Iq_reg.IP_REG_discharge = 0;          
          FOC_Iq_reg.IP_REG_lasterror=MC_BLDC_motor.Iqref-MC_BLDC_motor.Iqm;
          FOC_Id_reg.IP_REG_lasterror=-MC_BLDC_motor.Idm;
          FOC_rampup_step = FOC_rampup_step_fieldreg_loop;
      }
      break;

    case FOC_rampup_step_fieldreg_loop:
      FOC_rampup_step_fieldreg_counter++;
      Vqref=(int)((R*(long long int)MC_BLDC_motor.Iqref)>>31)+(187904750>>1);
      FOC_read_current();
      FOC_update_teta_speed();
      FOC_compute_clarke();
      FOC_compute_park();
      // REgulate Id
      FOC_Id_reg.IP_REG_mes = MC_BLDC_motor.Idm;
      FOC_Id_reg.IP_REG_ref = MC_BLDC_motor.Idref;
      IP_REG_compute(&FOC_Id_reg);
      Vdref = FOC_Id_reg.IP_REG_output;
      // REgulate Iq
      FOC_Iq_reg.IP_REG_mes = MC_BLDC_motor.Iqref;
      FOC_Iq_reg.IP_REG_ref = MC_BLDC_motor.Iqref;
      IP_REG_compute(&FOC_Iq_reg);
      Vq = FOC_Iq_reg.IP_REG_output;
      park_inv(Vdref,Vqref,MC_BLDC_motor.Tetam,&(svpwm_options.Valpha),&(svpwm_options.Vbeta));
      FOC_compute_svpwm();
      FOC_update_duty();
      if (FOC_rampup_step_fieldreg_counter == 10000) {
          MC_BLDC_motor.Iqref = IQREF_REGULAR;
          FOC_Iq_reg.IP_REG_lasterror = MC_BLDC_motor.Iqref-MC_BLDC_motor.Iqm;
          FOC_Id_reg.IP_REG_lasterror = -MC_BLDC_motor.Idm;
          FOC_Iq_reg.IP_REG_feedback = (Vqref>>4);
          FOC_Id_reg.IP_REG_discharge = 0;
          FOC_Iq_reg.IP_REG_discharge = 0;
          FOC_Speed_reg.IP_REG_discharge = 0;
          FOC_Iq_reg.Ki = (163040327>>5);
          FOC_Id_reg.Ki = (163040327>>5);            
          FOC_rampup_step = FOC_rampup_step_torquereg_loop;
      }
    break;


    case FOC_rampup_step_torquereg_loop:
      FOC_rampup_step_torquereg_counter++; 
      FOC_read_current();
      FOC_update_teta_speed();
      FOC_compute_clarke();
      FOC_compute_park();
      FOC_regulate_field();
      FOC_computeVdVq();
      park_inv(Vdref,Vqref,MC_BLDC_motor.Tetam,&(svpwm_options.Valpha),&(svpwm_options.Vbeta));
      FOC_compute_svpwm();
      FOC_update_duty();
      if (FOC_rampup_step_torquereg_counter == 10000) {
        MC_BLDC_motor.Speedref=MC_BLDC_motor.Speedm*transf_v;   
        FOC_Speed_reg.IP_REG_lasterror=0;  
        FOC_Speed_reg.IP_REG_feedback=(int)(((long long int)MC_BLDC_motor.Speedref*(long long int)FOC_Speed_reg.Kp)>>31) + (MC_BLDC_motor.Iqref/104);
        return (1);
      }
    break;
  }
  return (0);
}
