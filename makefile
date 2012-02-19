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

#+---------------------------------------------------------------------------
#
#  Copyright (c) 2010 Anton Gusev aka AHTOXA (HTTP://AHTOXA.NET)
#
#  File:       makefile
#
#  Contents:   makefile to build arm Cortex-M3 software with gcc
#
#----------------------------------------------------------------------------

#############  program name
	TARGET	= main

# program version
	VER_MAJOR	= 0
	VER_MINOR	= 1

	TOOL	= arm-none-eabi-
#	TOOL	= arm-kgp-eabi-

	OPTIMIZE	= -O2
	USE_LTO		= NO

# compile options 
#	MCU			= cortex-m3
# Select family 
# STM32F10X_LD    : STM32 Low density devices
# STM32F10X_LD_VL : STM32 Low density Value Line devices
# CHIP		= STM32F10X_MD    #: STM32 Medium density devices
# STM32F10X_MD_VL : STM32 Medium density Value Line devices
# STM32F10X_HD    : STM32 High density devices
# STM32F10X_HD_VL : STM32 XL-density devices
# STM32F10X_CL    : STM32 Connectivity line devices
# STM32F10X_XL    : STM32 XL-density devices
#	CHIP		= STM32F10X_MD

	MCU 	= cortex-m4
	CHIP	= STM32F407VG

#STARTUP = startup_stm32f10x_md.S
STARTUP = startup_$(CHIP).S

	RTOS_ROOT=../FreeRTOS
	CMSIS_DRIVER_DIR=../MyARMLib/CMSIS/include

#defines
	DEFS		= -D$(CHIP)
	DEFS		+= -DVER_MAJOR=$(VER_MAJOR)
	DEFS		+= -DVER_MINOR=$(VER_MINOR)

###########################################################
#  common part for all my cortex-m3 projects
###########################################################

	BASE		= .
	CC			= $(TOOL)gcc
	CXX			= $(TOOL)g++
	LD			= $(TOOL)g++
	AS			= $(CC) -x assembler-with-cpp
	OBJCOPY		= $(TOOL)objcopy
	OBJDUMP		= $(TOOL)objdump
	SIZE		= $(TOOL)size -d
	FLASHER		= openocd
	RM			= rm -f
	CP			= cp
	MD			= mkdir


#  dirs
	SRCDIR		= $(BASE)/src
	OBJDIR		= $(BASE)/obj
	EXEDIR		= $(BASE)/exe
	LSTDIR		= $(BASE)/lst
	PRJDIR		= $(BASE)/prj
	BAKDIR		= $(BASE)/bak

#files
	HEX		= $(EXEDIR)/$(TARGET).hex
	BIN		= $(EXEDIR)/$(TARGET).bin
	AXF		= $(EXEDIR)/$(TARGET).axf
	ELF		= $(EXEDIR)/$(TARGET).elf
	MAP		= $(LSTDIR)/$(TARGET).map
	LSS		= $(LSTDIR)/$(TARGET).lss
	OK		= $(EXEDIR)/$(TARGET).ok

# linker script (chip dependent)
	LD_SCRIPT	= $(SRCDIR)/$(CHIP).ld

# scmRTOS dir
	SCMDIR		= ../scmRTOS
	COMMON		= ../SamplesCommon
 
# source directories (all *.c, *.cpp and *.s files included)
	DIRS	:= $(SRCDIR)
	DIRS	+= $(COMMON)
	DIRS	+= $(RTOS_ROOT)/Source
	DIRS	+= $(RTOS_ROOT)/Source/portable/GCC/ARM_CM3
	DIRS	+= $(RTOS_ROOT)/Source/include
#	DIRS	+= ../MyARMLib/STM32/STM32F103_Pinboard_II
#	DIRS	+= $(RTOS_ROOT)/Demo/Common/Minimal
	DIRS	+= $(RTOS_ROOT)/Demo/Common/include
#	DIRS	+= $(RTOS_ROOT)/Demo/Common/ethernet/uIP/uip-1.0/uip 	
#	DIRS	+= $(RTOS_ROOT)/Source/portable/GCC/ARM_CM3_MPU	
	
	#STM32F1_DISCOVERY_DRIVER_DIR=../MyARMLib/STM32/STM32F4_discovery
	DIRS	+= $(CMSIS_DRIVER_DIR)
	DIRS	+= ../MyARMLib/CMSIS/include
#	DIRS	+= ../MyARMLib/STM32/STM32F10x/include
#	DIRS	+= ../MyARMLib/STM32/STM32F10x_StdPeriph_Driver/inc
#	DIRS	+= ../MyARMLib/STM32/STM32F10x_StdPeriph_Driver/src
	DIRS	+= ../MyARMLib/STM32/STM32F4xx/include
	DIRS	+= ../MyARMLib/STM32/STM32F4_discovery
	DIRS	+= ../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/src
	DIRS	+= ../MyARMLib/STM32/STM32F4xx_StdPeriph_Driver/inc


#	DIRS	+= ../MyARMLib/FreeRTOS

# includes
	INCS	:= $(patsubst %, -I "%", $(DIRS))

# individual source files
	SRCS	:= 

#calc obj files list
	OBJS	:= $(SRCS)
	OBJS	+= $(wildcard $(addsuffix /*.cpp, $(DIRS)))
	OBJS	+= $(wildcard $(addsuffix /*.c, $(DIRS)))
	OBJS	+= $(wildcard $(addsuffix /*.S, $(DIRS)))
	
	#OBJS	+= $(RTOS_ROOT)/Source/tasks.c
	#/port.c

#	OBJS	+= ../MyARMLib/STM32/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c 
#	OBJS	+= ../MyARMLib/STM32/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c 
#	OBJS	+= $(RTOS_ROOT)/Source/tasks.c
#	OBJS	+= $(RTOS_ROOT)/Source/list.c 
#	OBJS	+= $(RTOS_ROOT)/Source/timers.c 
#	OBJS	+= $(RTOS_ROOT)/Source/queue.c 
#	OBJS	+= $(RTOS_ROOT)/Source/tasks.c 
#	OBJS	+= $(RTOS_ROOT)/Source/portable/GCC/ARM_CM3/port.c 
#	OBJS	+= $(RTOS_ROOT)/Source/portable/MemMang/heap_2.c	
	OBJS	:= $(notdir $(OBJS))
	OBJS	:= $(OBJS:.cpp=.o)
	OBJS	:= $(OBJS:.c=.o)
	OBJS	:= $(OBJS:.S=.o)
	OBJS	:= $(patsubst %, $(OBJDIR)/%, $(OBJS))

#files to archive
	ARCFILES	= \
		$(SRCDIR) \
		$(PRJDIR) \
		$(SCMDIR) \
		$(BASE)/makefile \
		$(BASE)/.cproject \
		$(BASE)/.project

# flags
	FLAGS	= -mcpu=$(MCU) -mthumb
	FLAGS	+= $(INCS)
	FLAGS	+= -MD 
	#-DGCC_ARMCM3
	FLAGS	+= $(DEFS) -DUSE_STDPERIPH_DRIVER
	FLAGS	+= -Wa,-adhlns=$(addprefix $(LSTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))

	AFLAGS	= $(FLAGS)

	CFLAGS	= $(FLAGS)
	CFLAGS	+= $(OPTIMIZE)
	CFLAGS	+= -std=gnu99
	CFLAGS	+= -D GCC_ARMCM3
	CFLAGS	+= -g
	CFLAGS	+= -ffunction-sections -fdata-sections
#	CFLAGS	+= -Wall -Wextra
#	CFLAGS	+= -Wimplicit -Wcast-align -Wpointer-arith -Wredundant-decls
#	CFLAGS	+= -Wshadow -Wcast-qual -Wcast-align -Wnested-externs -pedantic

	CXXFLAGS	= $(FLAGS)
	CXXFLAGS	+= $(OPTIMIZE)
	CXXFLAGS	+= -g
	CXXFLAGS	+= -fno-exceptions -fno-rtti
	CXXFLAGS	+= -ffunction-sections -fdata-sections
	CXXFLAGS	+= -fno-threadsafe-statics
	CXXFLAGS	+= -funsigned-bitfields -fshort-enums
	CXXFLAGS	+= -Wall -Wextra
	CXXFLAGS	+= -Winline
	CXXFLAGS	+= -Wpointer-arith -Wredundant-decls
	CXXFLAGS	+= -Wshadow -Wcast-qual -Wcast-align -pedantic

	LD_FLAGS	= -mcpu=$(MCU)
	LD_FLAGS	+= -mthumb
	LD_FLAGS	+= -nostartfiles 
	LD_FLAGS	+= -Wl,-Map="$(MAP)",--cref
	LD_FLAGS	+= -Wl,--gc-sections
	LD_FLAGS	+= -T$(LD_SCRIPT)

ifeq ($(USE_LTO),YES)
	CFLAGS		+= -flto
	CXXFLAGS	+= -flto
	LD_FLAGS	+= -flto $(OPTIMIZE)
endif

#openocd command-line

# debug level (d0..d3)
	oocd_params		= -d0
# interface and board/target settings (using the OOCD target-library here)
#	oocd_params		+= -c "fast enable"
	oocd_params		+= -f interface/arm-usb-ocd.cfg 
	oocd_params		+= -f board/stm32f10x_128k_eval.cfg
	oocd_params		+= -c init -c targets
	oocd_params_program	= $(oocd_params)
# commands to prepare flash-write
	oocd_params_program	+= -c "halt"
# flash-write and -verify
	oocd_params_program	+= -c "flash write_image erase $(ELF)"
	oocd_params_program	+= -c "verify_image $(ELF)"
# reset target
	oocd_params_program	+= -c "reset run"
# terminate OOCD after programming
	oocd_params_program	+= -c shutdown

	oocd_params_reset	= $(oocd_params)
	oocd_params_reset	+= -c "reset run"
	oocd_params_reset	+= -c shutdown

.SILENT :

.PHONY: all start dirs build clean program reset archive

############# targets

all : start dirs $(AXF) $(ELF) $(HEX) $(BIN) $(LSS) $(OK)

build: clean all

start:
	@echo --- building $(TARGET) $(OBJS)

$(LSS): $(ELF) makefile
	@echo --- making asm-lst...
#	@$(OBJDUMP) -dStC $(ELF) > $(LSS)
	@$(OBJDUMP) -dC $(ELF) > $(LSS)

$(OK): $(ELF)
	@$(SIZE) $(ELF)
	@echo "Errors: none"

$(AXF):	$(OBJS) makefile
	@echo --- linking... axf
	$(LD) $(OBJS) $(LIBS) $(LD_FLAGS) -o "$(AXF)"
	
$(ELF):	$(OBJS) makefile
	@echo --- linking... 
	$(LD) $(OBJS) $(LIBS) $(LD_FLAGS) -o "$(ELF)"

$(HEX): $(ELF)
	@echo --- make hex...
	@$(OBJCOPY) -O ihex $(ELF) $(HEX)

$(BIN): $(ELF)
	@echo --- make binary...
	@$(OBJCOPY) -O binary $(ELF) $(BIN)

program: $(ELF)
	@echo "Programming with OPENOCD"
	$(FLASHER) $(oocd_params_program)

reset:
	@echo Resetting device
	$(FLASHER) $(oocd_params_reset)

VPATH := $(DIRS)

$(OBJDIR)/%.o: %.cpp makefile
	@echo --- compiling $<... 
	$(CXX) -c $(CXXFLAGS) -o $@ $< 

$(OBJDIR)/%.o: %.c makefile
	@echo --- compiling $<... 
	$(CC) -c $(CFLAGS) -o $@ $<

$(OBJDIR)/%.o: %.S makefile
	@echo --- assembling $<...
	$(AS) -c $(AFLAGS) -o $@ $<

dirs: $(OBJDIR) $(EXEDIR) $(LSTDIR) $(BAKDIR)

$(OBJDIR):
	-@$(MD) $(OBJDIR)

$(EXEDIR):
	-@$(MD) $(EXEDIR)

$(LSTDIR):
	-@$(MD) $(LSTDIR)

$(BAKDIR):
	-@$(MD) $(BAKDIR)

clean:
	-@$(RM) $(OBJDIR)/*.d 2>/dev/null
	-@$(RM) $(OBJDIR)/*.o 2>/dev/null
	-@$(RM) $(LSTDIR)/*.lst 2>/dev/null
	-@$(RM) $(ELF)
	-@$(RM) $(HEX)
	-@$(RM) $(LSS)
	-@$(RM) $(MAP)

archive:
	@echo --- archiving...
	7z a $(BAKDIR)/$(TARGET)_`date +%Y-%m-%d,%H-%M-%S` $(ARCFILES)
	@echo --- done!

# dependencies
ifeq (,$(findstring build,$(MAKECMDGOALS)))
 ifeq (,$(findstring clean,$(MAKECMDGOALS)))
  ifeq (,$(findstring dirs,$(MAKECMDGOALS)))
  -include $(wildcard $(OBJDIR)/*.d) 
  endif
 endif
endif
