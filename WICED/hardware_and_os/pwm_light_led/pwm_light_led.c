
//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------


// Use a PWM to adjust the brightness of LED1 on PSoC Shield based on light intensity
#include "wiced.h"

#define I2C_ADDRESS (0x42)
#define RETRIES (1)
#define DISABLE_DMA (WICED_TRUE)
#define NUM_MESSAGES (1)
#define LIGHT_REG 0x0F
#define PWM_PIN WICED_PWM_5

void application_start( )
{
	float duty_cycle = 0.0;

    wiced_init();	/* Initialize the WICED device */

    /* Setup I2C master */
    const wiced_i2c_device_t i2cDevice = {
        .port = WICED_I2C_2,
        .address = I2C_ADDRESS,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE
    };

    wiced_i2c_init(&i2cDevice);

    /* Tx buffer is used to set the offset */
    uint8_t tx_buffer[] = {LIGHT_REG};
    wiced_i2c_message_t setOffset;
    wiced_i2c_init_tx_message(&setOffset, tx_buffer, sizeof(tx_buffer), RETRIES, DISABLE_DMA);

    float light; //Variable to get light value
    wiced_i2c_message_t msg;
    wiced_i2c_init_rx_message(&msg, &light, sizeof(light), RETRIES, DISABLE_DMA);

    /* Initialize offset */
    wiced_i2c_transfer(&i2cDevice, &setOffset, NUM_MESSAGES);

    wiced_gpio_deinit(WICED_SH_LED1); // Need to de-init the GPIO if it is already set to drive the LED

    while ( 1 )
    {
        wiced_pwm_init(PWM_PIN, 1000, duty_cycle);
        wiced_pwm_start(PWM_PIN);

        wiced_i2c_transfer(&i2cDevice, &msg, NUM_MESSAGES); /* Get new data from I2C */
     	duty_cycle = (light*100)/500;

        wiced_rtos_delay_milliseconds( 200 );
    }
}
