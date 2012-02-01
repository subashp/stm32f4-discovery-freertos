#/*
#    FreeRTOS V7.1.0 - Copyright (C) 2011 Real Time Engineers Ltd.
#
#    ***************************************************************************
#    *                                                                         *
#    * If you are:                                                             *
#    *                                                                         *
#    *    + New to FreeRTOS,                                                   *
#    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
#    *    + Looking for basic training,                                        *
#    *    + Wanting to improve your FreeRTOS skills and productivity           *
#    *                                                                         *
#    * then take a look at the FreeRTOS books - available as PDF or paperback  *
#    *                                                                         *
#    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
#    *                  http://www.FreeRTOS.org/Documentation                  *
#    *                                                                         *
#    * A pdf reference manual is also available.  Both are usually delivered   *
#    * to your inbox within 20 minutes to two hours when purchased between 8am *
#    * and 8pm GMT (although please allow up to 24 hours in case of            *
#    * exceptional circumstances).  Thank you for your support!                *
#    *                                                                         *
#    ***************************************************************************
#
#    This file is part of the FreeRTOS distribution.
#
#    FreeRTOS is free software; you can redistribute it and/or modify it under
#    the terms of the GNU General Public License (version 2) as published by the
#    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
#    ***NOTE*** The exception to the GPL is included to allow you to distribute
#    a combined work that includes FreeRTOS without being obliged to provide the
#    source code for proprietary components outside of the FreeRTOS kernel.
#    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
#    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#    more details. You should have received a copy of the GNU General Public 
#    License and the FreeRTOS license exception along with FreeRTOS; if not it 
#    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
#    by writing to Richard Barry, contact details for whom are available on the
#    FreeRTOS WEB site.
#
#    1 tab == 4 spaces!
#
#    http://www.FreeRTOS.org - Documentation, latest information, license and
#    contact details.
#
#    http://www.SafeRTOS.com - A version that is certified for use in safety
#    critical systems.
#
#    http://www.OpenRTOS.com - Commercial support, development, porting,
#    licensing and training services.
#*/


#/************************************************************************* 
# * Please ensure to read http://www.freertos.org/portLM3Sxxxx_Eclipse.html
# * which provides information on configuring and running this demo for the
# * various Luminary Micro EKs.
# *************************************************************************/

RTOS_ROOT=d:/workspace/FreeRTOS

RTOS_SOURCE_DIR=$(RTOS_ROOT)/Source
DEMO_COMMON_DIR=$(RTOS_ROOT)/Demo/Common/Minimal
DEMO_INCLUDE_DIR=$(RTOS_ROOT)/Demo/Common/include
UIP_COMMON_DIR=$(RTOS_ROOT)/Demo/Common/ethernet/uIP/uip-1.0/uip
#STM32F4_DRIVER_DIR=$(RTOS_ROOT)/Common/drivers/ST

SRC_DIR=./RTOSDemo

CMSIS_DRIVER_DIR=../MyARMLib/CMSIS/Include
STM32F4_DISCOVERY_DRIVER_DIR=../MyARMLib/STM32/STM32F4_discovery
STM32F4_DRIVER_DIR=../MyARMLib/STM32/STM32F4xx/Include
STM32F4xx_StdPeriph_DRIVER_DIR=../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/Inc

MCU = cortex-m4
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
#CP=arm-none-eabi-objcopy
#OD=arm-none-eabi-objdump

OBJCOPY=arm-none-eabi-objcopy
#LDSCRIPT=$(SRC_DIR)/standalone.ld
#LDSCRIPT=$(SRC_DIR)/STM32F407xG.ld
LDSCRIPT=$(SRC_DIR)/stm32_flash.ld
# should use --gc-sections but the debugger does not seem to be able to cope with the option.
LINKER_FLAGS=-nostartfiles -Xlinker -oRTOSDemo.axf -Xlinker -M -Xlinker -Map=rtosdemo.map -Xlinker --no-gc-sections

DEBUG=-g
OPTIM=-O0


CFLAGS=$(DEBUG) \
		-I $(SRC_DIR) -I $(RTOS_SOURCE_DIR)/include -I $(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM3 \
		-I $(DEMO_INCLUDE_DIR) \
		-DUSE_STDPERIPH_DRIVER \
		-D STM32F407VG \
		-D inline= -mthumb \
		-mcpu=$(MCU) $(OPTIM) -T$(LDSCRIPT) \
		-D PACK_STRUCT_END=__attribute\(\(packed\)\)  \
		-D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
		-D sprintf=usprintf -D snprintf=usnprintf -D printf=uipprintf \
		-I $(UIP_COMMON_DIR)  -ffunction-sections -fdata-sections -I $(STM32F4_DRIVER_DIR) -I $(STM32F4_DISCOVERY_DRIVER_DIR) \
		-I $(STM32F4xx_StdPeriph_DRIVER_DIR)  -I $(CMSIS_DRIVER_DIR) \
		-DTARGET_IS_TEMPEST_RB1

#		../../Utilities/STM32F4-Discovery/stm32f4_discovery_audio_codec.c 
SOURCE=	$(SRC_DIR)/main.c $(SRC_DIR)/hw_config.c $(SRC_DIR)/system_stm32f4xx.c\
		../MyARMLib/STM32/STM32F4_discovery/stm32f4_discovery.c \
		../MyARMLib/STM32/STM32F4_discovery/stm32f4_discovery_lis302dl.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/misc.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
		../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
		$(RTOS_SOURCE_DIR)/list.c \
		$(RTOS_SOURCE_DIR)/timers.c \
		$(RTOS_SOURCE_DIR)/queue.c \
		$(RTOS_SOURCE_DIR)/tasks.c \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM3/port.c \
		$(RTOS_SOURCE_DIR)/portable/MemMang/heap_2.c

LIBS= 
#$(LUMINARY_DRIVER_DIR)/gcc/libdriver.a $(LUMINARY_DRIVER_DIR)/gcc/libgr.a

OBJS = $(SOURCE:.c=.o)

all: RTOSDemo.bin
	 
RTOSDemo.bin : RTOSDemo.axf
	$(OBJCOPY) RTOSDemo.axf -O binary RTOSDemo.bin

RTOSDemo.axf : $(OBJS) $(SRC_DIR)/startup_stm32f4xx.o Makefile
	$(CC) $(CFLAGS) $(OBJS) $(SRC_DIR)/startup_stm32f4xx.o $(LIBS) $(LINKER_FLAGS)

$(OBJS) : %.o : %.c Makefile $(SRC_DIR)/FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

startup_stm32f4xx.o : $(SRC_DIR)/startup_stm32f4xx.s Makefile
	$(CC) -c $(CFLAGS) -O1 $(SRC_DIR)/startup_stm32f4xx.s -o $(SRC_DIR)/startup.o
		
clean :
	touch Makefile
	cs-rm -f $(OBJS) $(SRC_DIR)/startup_stm32f4xx.o rtosdemo.map RTOSDemo.axf $(SRC_DIR)/RTOSDemo.bin
	



