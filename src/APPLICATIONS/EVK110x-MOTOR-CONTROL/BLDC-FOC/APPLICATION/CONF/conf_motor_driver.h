#ifndef __CONF_MOTOR_DRIVER_H__
#define __CONF_MOTOR_DRIVER_H__

#if BOARD == USB11
//~ #warning *** Including the correct file ***
#include "CONF/usb11.h"
#else
#include "CONF/mc300.h"
#endif

#endif /* __CONF_MOTOR_DRIVER_H__ */
