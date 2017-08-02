
//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------


// Use a PWM to adjust the brightness of LED1 on PSoC Shield
#include "wiced.h"

#define PWM_PIN WICED_PWM_5

void application_start( )
{
	float duty_cycle = 0.0;

    wiced_init();	/* Initialize the WICED device */

    wiced_gpio_deinit(WICED_SH_LED1); // Need to de-init the GPIO if it is already set to drive the LED

    while ( 1 )
    {
        wiced_pwm_init(PWM_PIN, 1000, duty_cycle);
        wiced_pwm_start(PWM_PIN);
        duty_cycle += 1.0;

        if(duty_cycle > 100.0)
        {
        	duty_cycle = 0.0;
        }
        wiced_rtos_delay_milliseconds( 20 );
    }
}
