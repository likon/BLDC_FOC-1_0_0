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

#define FALSE 0
#define TRUE 1

void tirq_init(void);

#endif /* __TIRQ_H__ */
