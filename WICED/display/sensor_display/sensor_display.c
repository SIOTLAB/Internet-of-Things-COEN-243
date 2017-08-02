//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------


/* Display sensor data from the analog co-processor on the OLED */
#include "u8g_arm.h"

#define RETRIES (1)
#define DISABLE_DMA (WICED_TRUE)
#define NUM_MESSAGES (1)
#define TEMPERATURE_REG 0x07

void application_start()
{

	/* Initialize the OLED display */
	wiced_i2c_device_t display_i2c =
    {
        .port          = WICED_I2C_2,
        .address       = 0x3C,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .flags         = 0,
        .speed_mode    = I2C_STANDARD_SPEED_MODE,

    };

    u8g_t display;
    u8g_init_wiced_i2c_device(&display_i2c);
    u8g_InitComFn(&display, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
    u8g_SetFont(&display, u8g_font_unifont);
    u8g_SetFontPosTop(&display);

    /* Initialize PSoC analog co-processor I2C interface and set the offset */
    const wiced_i2c_device_t psoc_i2c = {
    	.port 			= WICED_I2C_2,
		.address 		= 0x42,
		.address_width 	= I2C_ADDRESS_WIDTH_7BIT,
		.speed_mode 	= I2C_STANDARD_SPEED_MODE
    };

    wiced_i2c_init(&psoc_i2c);

    /* Tx buffer is used to set the offset */
    uint8_t tx_buffer[] = {TEMPERATURE_REG};
    wiced_i2c_message_t setOffset;
    wiced_i2c_init_tx_message(&setOffset, tx_buffer, sizeof(tx_buffer), RETRIES, DISABLE_DMA);

    /* Rx buffer is used to get temperature, humidity, light, and POT data - 4 bytes each */
    struct {
    	float temp;
		float humidity;
		float light;
		float pot;
    } rx_buffer;
    wiced_i2c_message_t msg;
    wiced_i2c_init_rx_message(&msg, &rx_buffer, sizeof(rx_buffer), RETRIES, DISABLE_DMA);

    /* Initialize offset */
    wiced_i2c_transfer(&psoc_i2c, &setOffset, NUM_MESSAGES);

    /* Strings to hold the results */
    char temp_str[18];
    char humidity_str[18];
    char light_str[18];
    char pot_str[18];

	while(1)
	{
		/* Get data from the PSoC */
		wiced_i2c_transfer(&psoc_i2c, &msg, NUM_MESSAGES); /* Get new data from I2C */

		/* Setup Display Strings */
		sprintf(temp_str,     "Temp:     %.1f", rx_buffer.temp);
		sprintf(humidity_str, "Humidity: %.1f", rx_buffer.humidity);
		sprintf(light_str,    "Light:    %.1f", rx_buffer.light);
		sprintf(pot_str,      "Pot:      %.1f", rx_buffer.pot);

		/* Send data to the display */
		u8g_FirstPage(&display);
		do {
			u8g_DrawStr(&display, 0, 0,  temp_str);
			u8g_DrawStr(&display, 0, 15, humidity_str);
			u8g_DrawStr(&display, 0, 30, light_str);
			u8g_DrawStr(&display, 0, 45, pot_str);
		} while (u8g_NextPage(&display));

		wiced_rtos_delay_milliseconds(500);
	}
}
