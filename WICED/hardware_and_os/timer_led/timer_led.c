
//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------

// Use an RTOS timer to control a blinking LED
#include "wiced.h"

#define TIMER_TIME (250)

/* Define the  function that will blink the LED on/off */
void ledBlink(void* arg)
{
	static wiced_bool_t led1 = WICED_FALSE;

	/* Toggle LED1 */
	if ( led1 == WICED_TRUE )
	{
		wiced_gpio_output_low( WICED_SH_LED1 );
		led1 = WICED_FALSE;
	}
	else
	{
		wiced_gpio_output_high( WICED_SH_LED1 );
		led1 = WICED_TRUE;
	}
}

void application_start( )
{
	wiced_timer_t timerHandle;

	wiced_init();	/* Initialize the WICED device */

	/* Initialize and start a timer */
    wiced_rtos_init_timer(&timerHandle, TIMER_TIME, ledBlink, NULL);
    wiced_rtos_start_timer(&timerHandle);

    while ( 1 )
    {
        wiced_rtos_delay_milliseconds(1000);
		/* Nothing needed here since we only have one thread. */
    }
}
