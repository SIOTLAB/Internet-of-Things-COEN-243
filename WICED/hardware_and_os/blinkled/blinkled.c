//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------



/* Blink LED1 and LED2 on the base board and LED1 of shield with a frequency of 2 Hz */
//------250ms (ON)------|------250ms(OFF)------|------250ms (ON)------|------250ms(OFF)------|

#include "wiced.h"

void application_start( )
{
    wiced_init();	/* Initialize the WICED device */

    /* The LED is initialized in platform.c.
     * If it was not, you would need the following:
     * wiced_gpio_init(WICED_SH_LED1, OUTPUT_PUSH_PULL); */

    while ( 1 )
    {
		/* LED off */
    	wiced_gpio_output_low( WICED_LED1 );
        wiced_gpio_output_low( WICED_LED2 );
        wiced_gpio_output_low( WICED_SH_LED1 );
    	wiced_rtos_delay_milliseconds( 250 );
		/* LED on */
    	wiced_gpio_output_high( WICED_LED1 );
        wiced_gpio_output_high( WICED_LED2 );
        wiced_gpio_output_high( WICED_SH_LED1 );
        wiced_rtos_delay_milliseconds( 250 );
    }
}
