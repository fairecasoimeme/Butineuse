# Application target name
TARGET = JN516xSniffer

# Default device type
JENNIC_CHIP ?= JN5168
JENNIC_CHIP_FAMILY     ?= JN516x

# Default SDK
JENNIC_SDK ?= JN-SW-4163

##############################################################################
# Select the network stack (e.g. MAC, ZBPRO)

JENNIC_STACK ?= None
###TODO: Check this

##############################################################################
# Debug options define DEBUG for HW debug
#DEBUG ?=HW
#
# Define which UART to use for debug
#DEBUG_PORT ?= UART0

#DEBUG ?= UART1

ifeq ($(DEBUG), UART1)
$(info Building with debug UART1...)
CFLAGS  += -DDBG_ENABLE
CFLAGS  += -DUART_DEBUGGING
CFLAGS  += -DDEBUG_UART=DBG_E_UART_1
CFLAGS  += -DTRACE_BEN=1
#CFLAGS  += -DDEBUG_APP_OTA
#CFLAGS  += -DDEBUG_ZCL
#CFLAGS	+= -DDEBUG_NWK_RECOVERY
#CFLAGS	+= -DDEBUG_PDM_EXTENDED
#CFLAGS	+= -DDEBUG_ZB_CONTROLBRIDGE_TASK
#CFLAGS  += -DTRACE_APP
#CFLAGS  += -DDEBUG_PDM_EXTENDED
#CFLAGS += -DDEBUG_SERIAL_LINK
endif

##############################################################################
# Define TRACE to use with DBG module
#TRACE ?=1

##############################################################################
# Path definitions

# Use if application directory contains multiple targets
SDK_BASE_DIR           = $(abspath ../../sdk/$(JENNIC_SDK))
#APP_BASE               = $(abspath ../..)
#APP_BLD_DIR            = $(APP_BASE)/Build/ZigbeeNodeControlBridge
#APP_SRC_DIR            = $(APP_BASE)/Source/ZigbeeNodeControlBridge
#ZBC_SRC_DIR            = $(COMPONENTS_BASE_DIR)/ZigbeeCommon/Source
#APP_COMMON_SRC_DIR     = $(APP_BASE)/Source/Common
#BDB_SRC_DIR            = $(COMPONENTS_BASE_DIR)/BDB/Source
#HWAPI_SRC_DIR          = $(COMPONENTS_BASE_DIR)/HardwareAPI/Source

#SDK_BASE_DIR       ?= $(abspath ../sdk/$(JENNIC_SDK)/)
SRC_DIR             = src
OBJ_DIR             = debug
#TOOL_COMMON_BASE_DIR= /usr/local
TOOL_COMMON_BASE_DIR= C:\NXP\bstudio_nxp\sdk\Tools



#TOOL_COMMON_BASE_DIR= $(HOME)/bin
TOOLCHAIN_PATH      = 
#TOOLCHAIN_PATH      = ba-elf-gcc-4.7.4
# Needs patch of sdk/JN-SW-4163/Chip/Common/Build/config_ba2.mk to not replace this value

##############################################################################
# Application Source files

APPSRC += main.c UartBuffered.c Queue.c Printf.c
#APPSRC += JN516xSniffer.c

# Specify additional Component libraries
LDLIBS += JPT_${JENNIC_CHIP} DBG_JN516x

##############################################################################
# Standard Application header search paths

INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ProductionTestApi/Include
INCFLAGS += -Iinc

##############################################################################
##############################################################################
# Configure for the selected chip or chip family

include $(SDK_BASE_DIR)/Chip/Common/Build/config.mk
include $(SDK_BASE_DIR)/Platform/Common/Build/Config.mk
include $(SDK_BASE_DIR)/Stack/Common/Build/config.mk

###TODO: change this
INCFLAGS += -I$(TOOL_COMMON_BASE_DIR)/$(TOOLCHAIN_PATH)/include
INCFLAGS += -I/usr/local/ba-elf/include
LDFLAGS += -L/usr/local/ba-elf/lib -fno-lto

##############################################################################

APPOBJS = $(addprefix $(OBJ_DIR)/,$(APPSRC:.c=.o))
APPSRCS = $(addprefix $(SRC_DIR)/,$(APPSRC))

##############################################################################
# Application dynamic dependencies

APPDEPS = $(APPOBJS:.o=.d)

#########################################################################
# Linker

# Add application libraries before chip specific libraries to linker so
# symbols are resolved correctly (i.e. ordering is significant for GCC)

LDLIBS := $(addsuffix _$(JENNIC_CHIP_FAMILY),$(APPLIBS)) $(LDLIBS)

#########################################################################
# Dependency rules

.PHONY: all clean
# Path to directories containing application source 
#vpath % $(APP_SRC_DIR)


all: $(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin

-include $(APPDEPS)
$(OBJ_DIR)/%.d:
	rm -f $(OBJ_DIR)/$*.o


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.S $(OBJ_DIR)/.dirok
	$(info Assembling $< ...)
	$(CC) -c -o $@ $(CFLAGS) $(INCFLAGS) $< -MMD -MF $(OBJ_DIR)/$*.d -MP
	@echo

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(OBJ_DIR)/.dirok
	$(info Compiling $< ...)
	$(CC) -c -o $@ $(CFLAGS) $(INCFLAGS) $< -MMD -MF $(OBJ_DIR)/$*.d -MP
	@echo

$(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).elf: $(APPOBJS) $(addsuffix _$(JENNIC_CHIP_FAMILY).a,$(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(APPLIBS))) 
	$(info Linking $@ ...)
	$(CC) -Wl,--gc-sections -Wl,-u_AppColdStart -Wl,-u_AppWarmStart $(LDFLAGS) -T$(LINKCMD) -o $@ $(APPOBJS) -Wl,--start-group  $(addprefix -l,$(LDLIBS)) -Wl,--end-group -Wl,-Map,$(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).map 
	${SIZE} $@
	@echo

$(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin: $(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).elf 
	$(info Generating binary ...)
	$(OBJCOPY) -S -O binary $< $@

$(OBJ_DIR)/.dirok:
	echo $@
	mkdir $(dir $@)
	touch $@

#########################################################################

clean:
#	rm -f $(APPOBJS) $(APPDEPS) $(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)*.bin $(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)*.elf $(OBJ_DIR)/$(TARGET)_$(JENNIC_CHIP)*.map
	rm -fR $(OBJ_DIR)

#########################################################################
