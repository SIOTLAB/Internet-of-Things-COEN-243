/*
* Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
* 
 * This software, associated documentation and materials ("Software"),
* is owned by Cypress Semiconductor Corporation
* or one of its subsidiaries ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/


/** @file
 * Defines internal configuration of the STM32 Discovery board
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************
 *  MCU Constants and Options
 *
 *  NOTE: The clock configuration utility from ST is used to calculate these values
 *        http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/utility/stsw-stm32090.zip
 ******************************************************/

/*  CPU clock : 120 MHz */
#define CPU_CLOCK_HZ         ( 96200000 )
#define DBG_WATCHDOG_TIMEOUT_MULTIPLIER ( 1875 )

/*  Use external crystal */
#define HSE_SOURCE           ( RCC_HSE_ON )

/*  AHB clock  : System clock */
#define AHB_CLOCK_DIVIDER    ( RCC_SYSCLK_Div1 )

/*  APB1 clock : AHB clock / 4 */
#define APB1_CLOCK_DIVIDER   ( RCC_HCLK_Div4 )

/*  APB2 clock : AHB clock / 2 */
#define APB2_CLOCK_DIVIDER   ( RCC_HCLK_Div2 )

/*  PLL source : external crystal */
#define PLL_SOURCE           ( RCC_PLLSource_HSE )

/*  PLLM : 8 */
#define PLL_M_CONSTANT       ( 15 )

/*  PLLN : 240 */
#define PLL_N_CONSTANT       ( 222 )

/*  PLLP : 2 */
#define PLL_P_CONSTANT       ( 4 )

/*  PLLQ : 5 */
#define PLL_Q_CONSTANT       ( 8 )

/*  System clock source  : PLL clock */
#define SYSTEM_CLOCK_SOURCE  ( RCC_SYSCLKSource_PLLCLK )

/*  SysTick clock source : AHB clock  */
#define SYSTICK_CLOCK_SOURCE ( SysTick_CLKSource_HCLK )

/*  Internal flash wait state : 3 cycles */
#define INT_FLASH_WAIT_STATE ( FLASH_Latency_3 )

/*  Internal flash voltage range : 2.7 to 3.6V */
#define PLATFORM_STM32_VOLTAGE_2V7_TO_3V6

/*  WICED Resources uses a filesystem */
#define USES_RESOURCE_FILESYSTEM

/* Location on SPI Flash where filesystem starts */
#define FILESYSTEM_BASE_ADDR  ( 0 )


/******************************************************
 *  Wi-Fi Options
 ******************************************************/

/*  GPIO pins are used to bootstrap Wi-Fi to SDIO or gSPI mode */
#define WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP

/*  Wi-Fi GPIO0 pin is used for out-of-band interrupt */
#define WICED_WIFI_OOB_IRQ_GPIO_PIN  ( 0 )

/*  Wi-Fi power pin is present */
#define WICED_USE_WIFI_POWER_PIN

/* Wi-Fi power pin is active high */
#define WICED_USE_WIFI_POWER_PIN_ACTIVE_HIGH

/*  Wi-Fi reset pin is present */
//#define WICED_USE_WIFI_RESET_PIN

/*  WLAN Powersave Clock Source */
#define WICED_USE_WIFI_32K_CLOCK_MCO

/*  OTA */
#define PLATFORM_HAS_OTA

/******************************************************
 *  Bluetooth Options
 ******************************************************/

/* Bluetooth device wake pin is present */
#define WICED_USE_BT_DEVICE_WAKE_PIN

/* Bluetooth host wake pin is present */
#define WICED_USE_BT_HOST_WAKE_PIN


#ifdef __cplusplus
} /* extern "C" */
#endif
