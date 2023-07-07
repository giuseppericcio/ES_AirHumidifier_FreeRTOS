
# How to use this makefile:
#
#  > make prepare  # copy all source files to a working directory (src)
#  > make          # build the default target (demo.bin)
#  > make debug    # run the GDB debugger
#  > make clean    # remove all temporary files in the working directories (src, obj, dep)

#  +++ CHECK ALL THE SETTINGS BELOW AND ADAPT THEM IF NEEDED +++

# default target and name of the image and executable files to generate
TARGET     = nofreertosproject

# path to the root folder of the STM32Cube platform
STM_DIR    = /c/Users/giuse/Documents/EmbeddedSystems/STM32CubeF3

# Board and MCU names as used in the linker script path and file name, e.g. "$(STM_DIR)/Demonstrations/SW4STM32/STM32F3-DISCO/STM32F307VGTx_FLASH.ld"
BOARD_UC   = STM32F3-DISCO
MCU_UC     = STM32F303VC

# board name as used in the STM32cube Drivers/BSP folder, e.g. "$(STM_DIR)/Drivers/BSP/STM32F3-Discovery"
#BSP_BOARD  = STM32F3-Discovery

# MCU name as used in the .s source file name, e.g. "startup_stm32f307xx.s"
MCU_LC     = stm32f303xc

# pre-processor symbol to be defined for the compilation (will be used in a -Dxxx flag in gcc)
MCU_MC     = STM32F303xC


###############################################################################
# Directories

HAL_DIR    = $(STM_DIR)/Drivers/STM32F3xx_HAL_Driver
CMSIS_DIR  = $(STM_DIR)/Drivers/CMSIS
DEV_DIR    = $(CMSIS_DIR)/Device/ST/STM32F3xx


###############################################################################
# Source files

SRCS = \
$(HAL_DIR)/Src/stm32f3xx_hal.c \
$(HAL_DIR)/Src/stm32f3xx_hal_cortex.c \
$(HAL_DIR)/Src/stm32f3xx_hal_gpio.c \
$(HAL_DIR)/Src/stm32f3xx_hal_dma.c \
$(HAL_DIR)/Src/stm32f3xx_hal_rcc.c \
$(HAL_DIR)/Src/stm32f3xx_hal_rcc_ex.c \
$(HAL_DIR)/Src/stm32f3xx_hal_spi.c \
$(HAL_DIR)/Src/stm32f3xx_hal_flash.c \
$(HAL_DIR)/Src/stm32f3xx_hal_pcd.c \
$(HAL_DIR)/Src/stm32f3xx_hal_adc.c \
$(HAL_DIR)/Src/stm32f3xx_hal_adc_ex.c \
$(HAL_DIR)/Src/stm32f3xx_hal_exti.c \
$(HAL_DIR)/Src/stm32f3xx_hal_tim.c \
$(HAL_DIR)/Src/stm32f3xx_hal_tim_ex.c \
Sensors/main.c \
Sensors/DHT11.c \
Sensors/stm32f3xx_hal_msp.c \
Sensors/stm32f3xx_it.c \
Sensors/system_stm32f3xx.c

# remove paths from the file names
SRCS_FN = $(notdir $(SRCS))

LDFILE     = Sensors/$(MCU_UC)Tx_FLASH.ld


###############################################################################
# Tools

PREFIX     = arm-none-eabi
CC         = $(PREFIX)-gcc
AR         = $(PREFIX)-ar
OBJCOPY    = $(PREFIX)-objcopy
OBJDUMP    = $(PREFIX)-objdump
SIZE       = $(PREFIX)-size
GDB        = $(PREFIX)-gdb

###############################################################################
# Options

# Defines (-D flags)
DEFS       = -D$(MCU_MC) -DUSE_HAL_DRIVER
DEFS       += -DUSE_DBPRINTF

# Include search paths (-I flags)
INCS       = -Isrc
INCS      += -I$(STM_DIR)/Drivers/CMSIS/Include
INCS      += -I$(STM_DIR)/Drivers/CMSIS/Device/ST/STM32F3xx/Include
INCS      += -I$(STM_DIR)/Drivers/STM32F3xx_HAL_Driver/Inc
#INCS      += -I$(STM_DIR)/Drivers/BSP/STM32F3-Discovery
#INCS      += -I$(STM_DIR)/Drivers/BSP/Components/lis302dl
#INCS      += -I$(STM_DIR)/Drivers/BSP/Components/lis3dsh

# Library search paths (-L flags)
LIBS       = -L$(CMSIS_DIR)/Lib

# Compiler flags
CFLAGS     = -Wall -g -std=c99 -Os
CFLAGS    += -mlittle-endian -mcpu=cortex-m4 -march=armv7e-m -mthumb
CFLAGS    += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS    += -ffunction-sections -fdata-sections -ftime-report
CFLAGS    += $(INCS) $(DEFS)

# Linker flags
LDFLAGS    = -Wl,--print-memory-usage -Wl,--gc-sections -Wl,-Map=$(TARGET).map $(LIBS) -Tsrc/linkerScript.ld

# Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon
#LDFLAGS   += --specs=nosys.specs --specs=nano.specs --specs=rdimon.specs -lc -lrdimon

# Source search paths
VPATH      = ./src
#VPATH     += $(BSP_DIR)
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source/

# Debugger flags
GDBFLAGS   =

# generate OBJS and DEPS target lists by prepending obj/ and dep prefixes
OBJS       = $(addprefix obj/,$(SRCS_FN:.c=.o))
DEPS       = $(addprefix dep/,$(SRCS_FN:.c=.d))


###################################################

.PHONY: all dirs debug prepare clean
	
all: $(TARGET).bin

-include $(DEPS)

dirs: dep obj

dep obj src:
	@echo "[MKDIR]   $@"
	mkdir -p $@

obj/%.o : %.c | dirs
	@echo "generating \"$@\" from \"$<\""
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF dep/$(*F).d

$(TARGET).elf: $(OBJS)
	@echo "[LD]      $(TARGET).elf"
	$(CC) $(CFLAGS) $(LDFLAGS) src/startup_$(MCU_LC).s $^ -o $@
	@echo "[OBJDUMP] $(TARGET).lst"
	$(OBJDUMP) -St $(TARGET).elf >$(TARGET).lst
	@echo "[SIZE]    $(TARGET).elf"
	$(SIZE) $(TARGET).elf

$(TARGET).bin: $(TARGET).elf
	@echo "[OBJCOPY] $(TARGET).bin"
	$(OBJCOPY) -O binary $< $@

debug:
	@if ! nc -z localhost 3333; then \
		@echo "\n\t[Error] OpenOCD is not running!\n"; exit 1; \
	else \
	$(GDB)  -ex "target extended localhost:3333" \
			-ex "monitor arm semihosting enable" \
			-ex "monitor reset halt" \
			-ex "load" \
			-ex "monitor reset init" \
			$(GDBFLAGS) $(TARGET).elf; \
	fi

prepare: src
	cp $(SRCS) src/
	cp $(STM_DIR)/Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/gcc/startup_$(MCU_LC).s src/
	cp $(LDFILE) src/linkerScript.ld
	cp Sensors/* src/

clean:
	@echo "[RM]      $(TARGET).bin"; rm -f $(TARGET).bin
	@echo "[RM]      $(TARGET).elf"; rm -f $(TARGET).elf
	@echo "[RM]      $(TARGET).map"; rm -f $(TARGET).map
	@echo "[RM]      $(TARGET).lst"; rm -f $(TARGET).lst
	@echo "[RM]      src files"; rm -f src/*
	@echo "[RM]      ld script"; rm -f src/linkerScript.ld
	@echo "[RMDIR]   dep"          ; rm -fr dep
	@echo "[RMDIR]   obj"          ; rm -fr obj
	@echo "[RMDIR]   obj"          ; rm -fr src
