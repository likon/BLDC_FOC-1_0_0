/**********************************************************************
 * File: conf_motor_driver.h
 * Author: Marcus Jansson
 * Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 * and Thomas Eberharter, Eberharter Elektronik GmbH
 *********************************************************************/

#ifndef __CONF_MOTOR_DRIVER_H__
#define __CONF_MOTOR_DRIVER_H__
//BOARD configuration is in src/BOARDS/xyz

#if BOARD == MC300
#include "CONF/mc300.h"
#else
#include "board.h"
#endif

#endif /* __CONF_MOTOR_DRIVER_H__ */
