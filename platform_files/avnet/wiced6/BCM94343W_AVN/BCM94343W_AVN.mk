#
# Copyright 2015, Broadcom Corporation
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Broadcom Corporation.
#

NAME := Platform_Avnet_Starter_Kit

WLAN_CHIP            := 4343W
WLAN_CHIP_REVISION   := A1
WLAN_CHIP_FAMILY     := 4343x
HOST_MCU_FAMILY      := STM32F4xx
HOST_MCU_VARIANT     := STM32F411
HOST_MCU_PART_NUMBER := STM32F411
BT_CHIP              := 43438
BT_CHIP_REVISION     := A1
BT_MODE              ?= HCI
BT_CHIP_XTAL_FREQUENCY := 37_4MHz

PLATFORM_SUPPORTS_BUTTONS := 1

INTERNAL_MEMORY_RESOURCES = $(ALL_RESOURCES)

ifndef BUS
BUS := SDIO
endif

EXTRA_TARGET_MAKEFILES +=  $(MAKEFILES_PATH)/standard_platform_targets.mk



VALID_BUSES := SDIO SPI

# Set the WIFI firmware in multi application file system to point to firmware
ifeq ($(NO_WIFI_FIRMWARE),)
MULTI_APP_WIFI_FIRMWARE   := resources/firmware/$(WLAN_CHIP)/$(WLAN_CHIP)$(WLAN_CHIP_REVISION)$(WLAN_CHIP_BIN_TYPE).bin
endif

ifeq ($(MULTI_APP_WIFI_FIRMWARE),)
ifeq ($(BUS),SDIO)
GLOBAL_DEFINES          += WWD_DIRECT_RESOURCES
endif
else
# Setting some internal build parameters
WIFI_FIRMWARE           := $(MULTI_APP_WIFI_FIRMWARE)
WIFI_FIRMWARE_LOCATION 	:= WIFI_FIRMWARE_IN_MULTI_APP
GLOBAL_DEFINES          += WIFI_FIRMWARE_IN_MULTI_APP
endif

# Global includes
GLOBAL_INCLUDES  := .
GLOBAL_INCLUDES  += $(WICED_BASE)/libraries/inputs/gpio_button

# Global defines
# HSE_VALUE = STM32 crystal frequency = 26MHz (needed to make UART work correctly)
GLOBAL_DEFINES += HSE_VALUE=26000000
GLOBAL_DEFINES += $$(if $$(NO_CRLF_STDIO_REPLACEMENT),,CRLF_STDIO_REPLACEMENT)

# Components
$(NAME)_COMPONENTS += drivers/spi_flash \
                      inputs/gpio_button

# Source files
$(NAME)_SOURCES := platform.c

# WICED APPS
# APP0 and FILESYSTEM_IMAGE are reserved main app and resources file system
# FR_APP := resources/sflash/snip_ota_fr-BCM943362WCD6.stripped.elf
# DCT_IMAGE :=
# OTA_APP :=
# FILESYSTEM_IMAGE :=
# WIFI_FIRMWARE :=
# APP0 :=
# APP1 :=
# APP2 :=

# WICED APPS LOOKUP TABLE
APPS_LUT_HEADER_LOC := 0x0000
APPS_START_SECTOR := 1

ifneq ($(APP),bootloader)
ifneq ($(MAIN_COMPONENT_PROCESSING),1)
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | IMPORTANT NOTES                                                                                     | )
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | Wi-Fi MAC Address                                                                                   | )
$(info |    The target Wi-Fi MAC address is defined in <WICED-SDK>/generated_mac_address.txt                 | )
$(info |    Ensure each target device has a unique address.                                                  | )
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | MCU & Wi-Fi Power Save                                                                              | )
$(info |    It is *critical* that applications using WICED Powersave API functions connect an accurate 32kHz | )
$(info |    reference clock to the sleep clock input pin of the WLAN chip. Please read the WICED Powersave   | )
$(info |    Application Note located in the documentation directory if you plan to use powersave features.   | )
$(info +-----------------------------------------------------------------------------------------------------+ )
endif
endif
