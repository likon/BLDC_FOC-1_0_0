/**********************************************************************
 * File: tirq.c
 * Author: Marcus Jansson
 * Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 * and Thomas Eberharter, Eberharter Elektronik GmbH
 *
 * This file is the replacement for the Hall detector code.
 *********************************************************************/

#include "tirq.h"

volatile unsigned short first_interrupt = 0;
volatile unsigned short nieme = 0;    //n:te
volatile U32 tirq_demi_period = tirq_demi_period_init ;
//~ volatile unsigned short teta0 = 0;
volatile U32 tirq_tj;
volatile U32 tirq_ti;                //!<  last period value

void tirq_estimator_update_teta_and_speed(volatile unsigned short *teta_elec, volatile unsigned short *vitesse_elec)
{
   nieme++;
   // 180*Fcpu*100e-6

  *teta_elec = (unsigned short)((U32)(vitesse_inst * 1 * nieme) / (U32)tirq_demi_period);
  if(*teta_elec > 360) {
    *teta_elec = 360;
    nieme = 0;
  }
  *vitesse_elec = (FCPU_HZ * PI) / tirq_demi_period;  //pi * Fcpu  (Fcpu=48Mhz)

  //~ printf("vitesse_elect: %i \n\r", vitesse_elec);	//TODO: REMOVE
  //~ printf("demi_period: %i, teta_elec: %i\n\r", tirq_demi_period, *teta_elec);
}

void tirq_estimator_init_teta(volatile unsigned short teta)
{
    nieme = (int)(((teta * tirq_demi_period) / vitesse_inst) + 2);
}
//------------------------------------------------------------------------------
/*! \name Initialization function
 */
//! @{
void tirq_estimator_init(void)
{
    nieme = 0;
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/*! \name Start function
 */
//! @{
void tirq_estimator_start(void)
{
    nieme = 0;
}

