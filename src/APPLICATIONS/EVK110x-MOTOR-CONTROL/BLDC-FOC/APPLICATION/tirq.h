/**********************************************************************
 * File: tirq.h
 * Author: Marcus Jansson
 * Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 * and Thomas Eberharter, Eberharter Elektronik GmbH
 *
 * This file contain code for timer interrupt, which is used for motor
 * speed regulation.
 *********************************************************************/

#ifndef __TIRQ_H__
#define __TIRQ_H__

#include <avr32/io.h>
#include "gpio.h"
#include "intc.h"
#include "tc.h"
#include "cycle_counter.h"
#include "conf_foc.h"

#define FALSE 0
#define TRUE 1

/*******************************************
 * Functions adapted from hall_estimator.c
 *******************************************/
/*! \name Initialization Functions
 */
//! @{
/*! \brief Init TIRQ ESTIMATOR
 * \param NONE.
 * \retval NONE
 */
extern void tirq_estimator_init(void);
/*! \brief Start TIRQ ESTIMATOR service
 * \param NONE.
 * \retval NONE
 */
extern void tirq_estimator_start(void);
/*! \brief Stop TIRQ ESTIMATOR service
 * \param NONE.
 * \retval NONE
 */
extern void tirq_estimator_stop(void);
/*! \brief Init interrupt for TIRQ ESTIMATOR service
 * \param NONE.
 * \retval NONE
 */
extern void tirq_estimator_init_interrupt(void);
/*! \brief Init Teta for TIRQ ESTIMATOR service
 * \param teta.Teta Param value to initialize for Position Estimator
 * \retval NONE
 */
extern void tirq_estimator_init_teta(volatile unsigned short teta);
//------------------------------------------------------------------------------
/*! \name Set/Get Functions
 */
//! @{
/*! \brief Update teta position/sector number value for HALL ESTIMATOR service
 * \param teta_elec : Electrical Position .
 * \param vitesse_elec : Electrical Speed .
 * \retval NONE
 */
extern void tirq_estimator_update_teta_and_speed(volatile unsigned short *teta_elec,
                                                 volatile unsigned short *vitesse_elec);

/*****************************************
 * Timer interrupt functions
 *****************************************/
void tirq_init(void);


#endif /* __TIRQ_H__ */
