#******************************************************************************
#
# Makefile - Rules for building the FreeRTOS example.
#
# Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=LM4F120H5QR

#
# Set the processor variant.
#
VARIANT=cm4f

#
# The base directory for StellarisWare.
#
ROOT=../../..

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../../../third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F
VPATH+=../../../third_party/FreeRTOS/Source/portable/MemMang/
VPATH+=../../../third_party/FreeRTOS/Source
VPATH+=../drivers
VPATH+=../../../utils
VPATH+=wearable_driver/
VPATH+=../hello/driver

#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=..
IPATH+=../../..
IPATH+=../../../third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F
IPATH+=../../../third_party/FreeRTOS
IPATH+=../../../third_party/FreeRTOS/Source/include
IPATH+=../../../third_party

#
# The default rule, which causes the FreeRTOS example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/freertos_demo.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the FreeRTOS example.
#
${COMPILER}/freertos_demo.axf: ${COMPILER}/buttons.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/freertos_demo.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/heap_2.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/accelerometer_task.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/list.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/port.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/queue.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/rgb.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/bluetooth_task.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/tasks.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/uartstdio.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/ustdlib.o
${COMPILER}/freertos_demo.axf: ${ROOT}/driverlib/${COMPILER}-cm4f/libdriver-cm4f.a
${COMPILER}/freertos_demo.axf: freertos_demo.ld
#${COMPILER}/freertos_demo.axf: ${COMPILER}/i2c.o
${COMPILER}/freertos_demo.axf: ${COMPILER}/Stellaris_API.o
SCATTERgcc_freertos_demo=freertos_demo.ld
ENTRY_freertos_demo=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RA1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
