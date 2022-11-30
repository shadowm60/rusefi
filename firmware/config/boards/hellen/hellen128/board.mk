# Combine the related files for a specific platform and MCU.

# Target ECU board design
BOARDCPPSRC = $(BOARDS_DIR)/hellen/hellen128/board_configuration.cpp

BOARDINC = $(BOARDS_DIR)/hellen/hellen128

# Set this if you want a default engine type other than normal hellen128
ifeq ($(VAR_DEF_ENGINE_TYPE),)
  VAR_DEF_ENGINE_TYPE = -DDEFAULT_ENGINE_TYPE=HELLEN_128_MERCEDES_4_CYL
endif


DDEFS += -DEFI_MAIN_RELAY_CONTROL=TRUE

# Disable serial ports on this board as UART3 causes a DMA conflict with the SD card
DDEFS += -DTS_NO_PRIMARY=1

# Add them all together
DDEFS += -DFIRMWARE_ID=\"hellen128\" $(VAR_DEF_ENGINE_TYPE)
DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE -DSTM32_ADC_USE_ADC3=TRUE

DDEFS += -DSHORT_BOARD_NAME=hellen128

# take over and modify some aspects from these mk files
# doing so hellen128 can be a bit different and we do not need to modify all others 
#include $(BOARDS_DIR)/hellen/hellen-common176.mk
#include $(BOARDS_DIR)/hellen/hellen-common.mk

# 176 package MCU
ifeq ($(LED_CRITICAL_ERROR_BRAIN_PIN),)
  LED_CRITICAL_ERROR_BRAIN_PIN = -DLED_CRITICAL_ERROR_BRAIN_PIN=H176_LED1_RED
endif

DDEFS += $(LED_CRITICAL_ERROR_BRAIN_PIN)

# Combine the related files for a specific platform and MCU.

# Target ECU board design
BOARDCPPSRC += $(BOARDS_DIR)/hellen/hellen_common.cpp \
    $(BOARDS_DIR)/hellen/hellen_board_id.cpp

DDEFS += -DLED_ERROR_BRAIN_PIN_MODE=INVERTED_OUTPUT
DDEFS += -DLED_RUNING_BRAIN_PIN_MODE=INVERTED_OUTPUT
DDEFS += -DLED_WARNING_BRAIN_PIN_MODE=INVERTED_OUTPUT
DDEFS += -DLED_COMMUNICATION_BRAIN_PIN_MODE=INVERTED_OUTPUT

# We are running on Hellen-One hardware!
DDEFS += -DHW_HELLEN=1

#all this modification to modify this 
#DDEFS += -DTS_NO_SECONDARY=TRUE
# DDEFS += -DEFI_CONSOLE_TX_BRAIN_PIN=Gpio::E1 -DEFI_CONSOLE_RX_BRAIN_PIN=Gpio::E0
DDEFS += -DHAL_USE_SERIAL=TRUE
DDEFS += -DSTM32_SERIAL_USE_UART8=TRUE
DDEFS += -DTS_SECONDARY_PORT=SD8
DDEFS += -DEFI_USE_UART_DMA=TRUE
#we need this to have UART8
#IS_STM32F429 = yes -> true for revc, would break reva, not tested on revb
IS_H128 = yes



