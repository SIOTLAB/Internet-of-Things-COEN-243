
//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------

// Use a thread to blink an LED with the period received from user as a 3 digit number

#include "wiced.h"

#define RX_BUFFER_SIZE (5)

/* Thread parameters */
#define THREAD_PRIORITY 	(10)
#define THREAD_STACK_SIZE	(1024)

volatile uint16_t interval; //LED blinking interval

/* Define the thread function that will blink the LED on/off every 500ms */
void ledThread(wiced_thread_arg_t arg)
{
    wiced_bool_t led1 = WICED_FALSE;

    while(1)
    {
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
        wiced_rtos_delay_milliseconds( interval );
    }
}

void application_start( )
{
    uint32_t expected_data_size = 3; //Receive 3 digits from user
    char    receiveString[expected_data_size];
    interval = 250;

    wiced_thread_t ledThreadHandle;

    wiced_init();	/* Initialize the WICED device */

    /* Configure and start the UART. */
    wiced_ring_buffer_t rx_buffer;
    uint8_t             rx_data[RX_BUFFER_SIZE];
    ring_buffer_init(&rx_buffer, rx_data, RX_BUFFER_SIZE ); /* Initialize ring buffer to hold receive data */

    /* Note that WICED_DISABLE_STDIO must be defined in the make file for this to work */
    wiced_uart_config_t uart_config =
    {
            .baud_rate    = 115200,
            .data_width   = DATA_WIDTH_8BIT,
            .parity       = NO_PARITY,
            .stop_bits    = STOP_BITS_1,
            .flow_control = FLOW_CONTROL_DISABLED,
    };

    wiced_uart_init( STDIO_UART, &uart_config, &rx_buffer); /* Setup UART */

    /* Initialize and start a new thread */
    wiced_rtos_create_thread(&ledThreadHandle, THREAD_PRIORITY, "ledThread", ledThread, THREAD_STACK_SIZE, NULL);

    wiced_uart_transmit_bytes(WICED_UART_1, "Enter a value between 000 and 999:\n", 35);

    while ( 1 ) {
        if ( wiced_uart_receive_bytes( STDIO_UART, &receiveString, &expected_data_size, WICED_NEVER_TIMEOUT ) == WICED_SUCCESS )
        {
            interval = atoi(receiveString); //ASCI to integer conversion
            wiced_uart_transmit_bytes(WICED_UART_1, "\nNew Value entered: " , 21);
            wiced_uart_transmit_bytes(WICED_UART_1, &receiveString , 3);
            wiced_uart_transmit_bytes(WICED_UART_1, "\n" , 1);
        }
    }
}
