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
 * Defines peripherals available for use on Avnet AWS combo board
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
/*
BCM94343W_AVN platform pin definitions
*/

/* Pin Definitions
 *
	PCB Module        |  SIP PIN			|   MCU PIN	|		WICED PIN	  |
	----------------------------------------|-----------|---------------------|
	MICRO_WKUP		  |  MICRO_WKUP			|   PA0		|		WICED_GPIO_1  |
	MICRO_ADC_IN1	  |  MICRO_ADC_IN1		|   PA1		|		WICED_GPIO_2  |
	MICRO_UART_RX	  |  USART1_RX			|   PA10	|		WICED_GPIO_10 |
	MICRO_UART_CTS	  |  USART1_CTS			|   PA11	|		WICED_GPIO_15 |
	MICRO_UART_RTS	  |  USART1_RTS			|   PA12	|		WICED_GPIO_16 |
	MICRO_UART_TX	  |  USART1_TX			|   PA9		|		WICED_GPIO_9  |
	MICRO_ADC_IN2	  |  MICRO_ADC_IN2		|   PA2		|		WICED_GPIO_3  |
	MICRO_ADC_IN3	  |  MICRO_ADC_IN3		|   PA3		|		WICED_GPIO_4  |
	MICRO_GPIO_0	  |  MICRO_GPIO_5		|   PB0		|		WICED_GPIO_29 | 
	MICRO_I2C2_SCL	  |  I2C2_SCL			|   PB10	|		WICED_GPIO_20 |
	MICRO_I2C2_SDA	  |  I2C2_SDA			|   PB11	|		WICED_GPIO_21 |
	MICRO_GPIO_6	  |  I2S2_WS			|   PB9		|		WICED_GPIO_19 |
	MICRO_GPIO_B	  |  I2S2_SD			|   PC3		|		WICED_GPIO_17 |
	MICRO_SPI2_SSN	  |  MICRO_GPIO_32		|   PB12	|		WICED_GPIO_22 |
	MICRO_SPI2_SCK	  |  MICRO_GPIO_33		|   PB13	|		WICED_GPIO_23 |
	MICRO_SPI2_MISO	  |  MICRO_GPIO_34		|   PB14	|		WICED_GPIO_24 |
	MICRO_SPI2_MOSI	  |  MICRO_GPIO_3		|   PB15	|		WICED_GPIO_25 |
	MICRO_I2C1_SCL	  |  I2C1_SCL			|   PB6		|		WICED_GPIO_11 |
	MICRO_I2C1_SDA	  |  I2C1_SDA			|   PB7		|		WICED_GPIO_12 |
	MICRO_GPIO_1	  |  MICRO_GPIO_28		|   PB8		|		WICED_GPIO_18 | 
	MICRO_GPIO_2	  |  MICRO_GPIO_1		|   PC0		|		WICED_GPIO_26 |
	MICRO_GPIO_3	  |  MICRO_GPIO_2		|   PC1		|		WICED_GPIO_27 |
	MICRO_GPIO_4	  |  MICRO_17			|   PC2		|		WICED_GPIO_28 |
	MICRO_ADC_IN15	  |  MICRO_GPIO_4		|   PC5		|		WICED_GPIO_30 |
	USART6_TX_I2S2_MCK|  USART6_TX			|   PC6		|		WICED_GPIO_13 |
	USART6_RX_I2S2_CK |  USART6_RX			|   PC7		|		WICED_GPIO_14 |
	WIFI_GPIO_1		  |  WIFI_GPIO_1		|   PD13	|       WICED_GPIO_23 |
	----------------------------------------|-----------|---------------------|
	MICRO_SPI_SSN	  |  MICRO_SPI_NSS		|	PA4		|		WICED_GPIO_5  |# Following pins are connected to the SPI flash
	MICRO_SPI_SCK	  |  MICRO_SPI_SCK		|	PA5		|		WICED_GPIO_6  | #SPI_FLASH_CLK
	MICRO_SPI_MISO	  |  MICRO_SPI_MISO		|	PA6		|		WICED_GPIO_7  | #SPI_FLASH_MISO
	MICRO_SPI_MOSI	  |  MICRO_SPI_MOSI		|	PA7		|		WICED_GPIO_8  | #SPI_FLASH_MOSI
*
*/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WICED_GPIO_1,
    WICED_GPIO_2,
    WICED_GPIO_3,
    WICED_GPIO_4,
    WICED_GPIO_5,
    WICED_GPIO_6,
    WICED_GPIO_7,
    WICED_GPIO_8,
    WICED_GPIO_9,
    WICED_GPIO_10,
    WICED_GPIO_11,
    WICED_GPIO_12,
    WICED_GPIO_13,
    WICED_GPIO_14,
    WICED_GPIO_15,
    WICED_GPIO_16,
    WICED_GPIO_17,
    WICED_GPIO_18,
    WICED_GPIO_19,
    WICED_GPIO_20,
    WICED_GPIO_21,
    WICED_GPIO_22,
    WICED_GPIO_23,
    WICED_GPIO_24,
    WICED_GPIO_25,
    WICED_GPIO_26,
    WICED_GPIO_27,
    WICED_GPIO_28,
    WICED_GPIO_29,
    WICED_GPIO_30,
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    WICED_GPIO_32BIT = 0x7FFFFFFF,
} wiced_gpio_t;

typedef enum
{
    WICED_SPI_1,
    WICED_SPI_2,
    WICED_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
    WICED_SPI_32BIT = 0x7FFFFFFF,
} wiced_spi_t;

typedef enum
{
    WICED_I2C_1,
    WICED_I2C_2,
    WICED_I2C_MAX,
    WICED_I2C_32BIT = 0x7FFFFFFF,
} wiced_i2c_t;

typedef enum
{
    WICED_I2S_NONE,
    WICED_I2S_MAX, /* Denotes the total number of I2S port aliases.  Not a valid I2S alias */
    WICED_I2S_32BIT = 0x7FFFFFFF
} wiced_i2s_t;

typedef enum
{
    WICED_PWM_1,
    WICED_PWM_2,
    WICED_PWM_3,
    WICED_PWM_4,
    WICED_PWM_5,
    WICED_PWM_6,
    WICED_PWM_7,
    WICED_PWM_8,
    WICED_PWM_9,
    WICED_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    WICED_PWM_32BIT = 0x7FFFFFFF,
} wiced_pwm_t;

typedef enum
{
    WICED_ADC_1,
    WICED_ADC_2,
    WICED_ADC_3,
    WICED_ADC_4,
    WICED_ADC_MAX, /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    WICED_ADC_32BIT = 0x7FFFFFFF,
} wiced_adc_t;

typedef enum
{
    WICED_UART_1,
    WICED_UART_2,
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    WICED_UART_32BIT = 0x7FFFFFFF,
} wiced_uart_t;

/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button Alias */
} platform_button_t;

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_PLATFORM_BUTTON_COUNT  ( 1 )

/* UART port used for standard I/O */
#define STDIO_UART ( WICED_UART_1 )

/* SPI flash is present */
#define WICED_PLATFORM_INCLUDES_SPI_FLASH
#define WICED_SPI_FLASH_CS  ( WICED_GPIO_5 )

/* Components connected to external I/Os */
#define PLATFORM_LED_COUNT   ( 2 )
#define WICED_LED1           ( WICED_GPIO_29 )
#define WICED_LED2           ( WICED_GPIO_18 )
#define WICED_LED1_ON_STATE  ( WICED_ACTIVE_HIGH )
#define WICED_LED2_ON_STATE  ( WICED_ACTIVE_HIGH )
#define WICED_BUTTON1        ( WICED_GPIO_19 )

#define WICED_LIGHT          ( WICED_ADC_4 )

/* Bootloader OTA and OTA2 factory reset during settings */
#define PLATFORM_FACTORY_RESET_BUTTON_INDEX     ( PLATFORM_BUTTON_1 )
// #define PLATFORM_FACTORY_RESET_BUTTON_GPIO      ( WICED_BUTTON1 )
#define PLATFORM_FACTORY_RESET_TIMEOUT          ( 10000 )

/* Generic button checking defines */
#define PLATFORM_BUTTON_PRESS_CHECK_PERIOD      ( 100 )
#define PLATFORM_BUTTON_PRESSED_STATE           (   0 )

#define PLATFORM_GREEN_LED_INDEX                ( WICED_LED_INDEX_1 )
#define PLATFORM_RED_LED_INDEX                  ( WICED_LED_INDEX_2 )

/*  Bootloader OTA/OTA2 LED to flash while "Factory Reset" button held */
#define PLATFORM_FACTORY_RESET_LED_GPIO      ( WICED_LED1 )
#define PLATFORM_FACTORY_RESET_LED_ON_STATE  ( WICED_LED1_ON_STATE )

#ifdef __cplusplus
} /*extern "C" */
#endif
