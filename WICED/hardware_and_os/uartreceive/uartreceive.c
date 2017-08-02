
//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------


// Receive characters from the UART interface. 0 turns the LED off, 1 turns the LED on
#include "wiced.h"

#define RX_BUFFER_SIZE (5)

/* Main application */
void application_start( )
{
	char    receiveChar;
    uint32_t expected_data_size = 1;

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

    wiced_uart_init( WICED_UART_1, &uart_config, &rx_buffer); /* Setup UART */

    while ( 1 )
    {
        if ( wiced_uart_receive_bytes( WICED_UART_1, &receiveChar, &expected_data_size, WICED_NEVER_TIMEOUT ) == WICED_SUCCESS )
        {
            /* If we get here then a character has been received */
        	if(receiveChar == '0') /* Turn LED off */
        	{
        		wiced_gpio_output_low( WICED_LED1 );
        	}
        	if(receiveChar == '1') /* Turn LED on */
        	{
        		wiced_gpio_output_high( WICED_LED1 );
        	}
        }

    }
}
