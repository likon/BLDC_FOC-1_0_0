# Hey Emacs, this is a -*- makefile -*-

# The purpose of this file is to define the build configuration variables used
# by the generic Makefile. See Makefile header for further information.

# Copyright (C) 2006-2008, Atmel Corporation All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/
# or other materials provided with the distribution.
#
# 3. The name of ATMEL may not be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
# SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Base paths
PRJ_PATH = src
APPS_PATH = $(PRJ_PATH)/APPLICATIONS
MAIN_PATH = $(PRJ_PATH)/APPLICATIONS/EVK110x-MOTOR-CONTROL/BLDC-FOC/APPLICATION
BRDS_PATH = $(PRJ_PATH)/BOARDS
COMP_PATH = $(PRJ_PATH)/COMPONENTS
DRVR_PATH = $(PRJ_PATH)/DRIVERS
SERV_PATH = $(PRJ_PATH)/SERVICES
UTIL_PATH = $(PRJ_PATH)/UTILS


# CPU architecture: {ap|ucr1|ucr2}
ARCH = ucr2

# Part: {none|ap7xxx|uc3xxxxx}
PART = uc3a1512

# Flash memories: [{cfi|internal}@address,size]...
FLASH = internal@0x80000000,512Kb

# Clock source to use when programming: [{xtal|extclk|int}]
PROG_CLOCK = xtal

# Device/Platform/Board include path
PLATFORM_INC_PATH = \
  $(BRDS_PATH)/

# Target name: {*.a|*.elf}
TARGET = $(PART)-bldc_foc.elf

# Definitions: [-D name[=definition]...] [-U name...]
# Things that might be added to DEFS:
#   BOARD             Board used: see $(BRDS_PATH)/board.h
#   EXT_BOARD         Extension board used (if any): see $(BRDS_PATH)/board.h
DEFS = -D BOARD=USB11

# Include path
INC_PATH = \
  $(UTIL_PATH)/ \
  $(UTIL_PATH)/PREPROCESSOR/ \
  $(SERV_PATH)/USB/CLASS/DFU/EXAMPLES/ISP/BOOT/ \
  $(UTIL_PATH)/LIBS/NEWLIB_ADDONS/INCLUDE/ \
  $(UTIL_PATH)/LIBS/DRIVERS/AT32UC3B/INCLUDE/ \
  $(UTIL_PATH)/DEBUG/ \
  $(DRVR_PATH)/USBB/ \
  $(DRVR_PATH)/USBB/ENUM/ \
  $(DRVR_PATH)/USBB/ENUM/DEVICE/ \
  $(SERV_PATH)/USB/ \
  $(SERV_PATH)/USB/CLASS/CDC/ \
  $(SERV_PATH)/MOTOR_CONTROL/CONCORDIA/ \
  $(SERV_PATH)/MOTOR_CONTROL/HALL_ESTIMATOR/ \
  $(SERV_PATH)/MOTOR_CONTROL/MOTOR_DRIVER/BLDC_MOTOR/ \
  $(SERV_PATH)/MOTOR_CONTROL/PARK/ \
  $(SERV_PATH)/MOTOR_CONTROL/REGULATOR/ \
  $(SERV_PATH)/MOTOR_CONTROL/SVPWM/ \
  $(MAIN_PATH)/CONF/ \
  $(MAIN_PATH)/ENUM \
  $(MAIN_PATH)/ \
#  ../../../ \
#  ../../

# C source files
CSRCS = \
  $(UTIL_PATH)/DEBUG/print_funcs.c \
  $(DRVR_PATH)/USBB/usb_drv.c \
  $(DRVR_PATH)/USBB/ENUM/usb_task.c \
  $(DRVR_PATH)/USBB/ENUM/DEVICE/usb_device_task.c \
  $(DRVR_PATH)/USBB/ENUM/DEVICE/usb_standard_request.c \
  $(MAIN_PATH)/ENUM/usb_descriptors.c \
  $(MAIN_PATH)/ENUM/usb_specific_request.c \
    $(SERV_PATH)/MOTOR_CONTROL/CONCORDIA/concordia.c \
    $(SERV_PATH)/MOTOR_CONTROL/HALL_ESTIMATOR/hall_estimator.c \
    $(SERV_PATH)/MOTOR_CONTROL/MOTOR_DRIVER/BLDC_MOTOR/pwm_drv.c \
    $(SERV_PATH)/MOTOR_CONTROL/PARK/park.c \
    $(SERV_PATH)/MOTOR_CONTROL/REGULATOR/ip_reg.c \
    $(SERV_PATH)/MOTOR_CONTROL/SVPWM/svpwm.c \
  $(MAIN_PATH)/delay.c \
  $(MAIN_PATH)/device_cdc_task.c \
  $(MAIN_PATH)/main.c \
  $(MAIN_PATH)/mc_control.c \
  $(MAIN_PATH)/mc_driver.c \
  $(MAIN_PATH)/foc.c \
  $(MAIN_PATH)/uart_usb_lib.c \
  $(MAIN_PATH)/util.c

# Assembler source files
ASSRCS = \
  $(SERV_PATH)/USB/CLASS/DFU/EXAMPLES/ISP/BOOT/trampoline.S \
  $(DRVR_PATH)/INTC/exception.S

# Library path
LIB_PATH = \
   $(UTIL_PATH)/LIBS/DRIVERS/AT32UC3A/GCC/ \
   $(UTIL_PATH)/LIBS/NEWLIB_ADDONS/AT32UCR2/

# Libraries to link with the project
LIBS = \
  drivers-at32uc3a-speed_opt \
  newlib_addons-at32ucr2-speed_opt

# Linker script file if any
LINKER_SCRIPT = $(UTIL_PATH)/LINKER_SCRIPTS/AT32UC3A/0512/GCC/link_uc3a0512.lds

# Options to request or suppress warnings: [-fsyntax-only] [-pedantic[-errors]] [-w] [-Wwarning...]
# For further details, refer to the chapter "GCC Command Options" of the GCC manual.
WARNINGS = -Wall

# Options for debugging: [-g]...
# For further details, refer to the chapter "GCC Command Options" of the GCC manual.
DEBUG = -g

# Options that control optimization: [-O[0|1|2|3|s]]...
# For further details, refer to the chapter "GCC Command Options" of the GCC manual.
OPTIMIZATION = -O3 -fno-strict-aliasing -ffunction-sections -fdata-sections

# Extra flags to use when preprocessing
CPP_EXTRA_FLAGS =

# Extra flags to use when compiling
C_EXTRA_FLAGS =

# Extra flags to use when assembling
AS_EXTRA_FLAGS =

# Extra flags to use when linking
LD_EXTRA_FLAGS = -Wl,--gc-sections -Wl,-e,_trampoline

# Documentation path
DOC_PATH = \
  $(MAIN_PATH)/DOC/

# Documentation configuration file
DOC_CFG = \
  doxyfile.doxygen
