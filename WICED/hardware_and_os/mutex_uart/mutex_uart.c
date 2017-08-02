
//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------



// Use a MUTEX to lock access to the WPRINT function.
// Verify that printing works as expected on a terminal window.
// Without the mutex, some of the printing may not be correct.
#include "wiced.h"

// Comment out the following line to see what happens without the mutex
//#define USE_MUTEX

/* Thread parameters */
#define THREAD_PRIORITY 	(10)
#define THREAD_STACK_SIZE	(1024)

static wiced_mutex_t printMutexHandle;
static volatile uint16_t counter;

/* Define the thread function that will blink LED1 on/off every 250ms */
void led1Thread(wiced_thread_arg_t arg)
{
	wiced_bool_t led1 = WICED_FALSE;

	while(1)
	{
		#ifdef USE_MUTEX
			wiced_rtos_lock_mutex(&printMutexHandle);
		#endif
		WPRINT_APP_INFO(("TOGGLE LED1 - %d \n", counter++));
		#ifdef USE_MUTEX
			wiced_rtos_unlock_mutex(&printMutexHandle);
		#endif
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
		wiced_rtos_delay_milliseconds( 250 );
	}
}

/* Define the thread function that will blink LED0 on/off every 249ms */
void led0Thread(wiced_thread_arg_t arg)
{
	wiced_bool_t led0 = WICED_FALSE;

	while(1)
	{
		#ifdef USE_MUTEX
			wiced_rtos_lock_mutex(&printMutexHandle);
		#endif
	        WPRINT_APP_INFO(("TOGGLE LED0 - %d \n", counter++));
		#ifdef USE_MUTEX
			wiced_rtos_unlock_mutex(&printMutexHandle);
		#endif

		/* Toggle LED2 */
		if ( led0 == WICED_TRUE )
		{
			wiced_gpio_output_low( WICED_SH_LED0 );
			led0 = WICED_FALSE;
		}
		else
		{
			wiced_gpio_output_high( WICED_SH_LED0 );
			led0 = WICED_TRUE;
		}
		wiced_rtos_delay_milliseconds( 250 );
	}
}

void application_start( )
{
    counter = 0;
	wiced_thread_t led1ThreadHandle;
	wiced_thread_t led0ThreadHandle;


	wiced_init();	/* Initialize the WICED device */

	/* Initialize the Mutex */
    wiced_rtos_init_mutex(&printMutexHandle);

	/* Initialize and start threads */
    wiced_rtos_create_thread(&led1ThreadHandle, THREAD_PRIORITY, "led1Thread", led1Thread, THREAD_STACK_SIZE, NULL);
    wiced_rtos_create_thread(&led0ThreadHandle, THREAD_PRIORITY, "led0Thread", led0Thread, THREAD_STACK_SIZE, NULL);

    /* No while(1) here since everything is done by the new threads. */
}
