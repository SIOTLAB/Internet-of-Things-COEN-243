//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------



#include "u8g_arm.h"

void application_start()
{
    u8g_t u8g; // 1: setup a structure

    wiced_i2c_device_t oled_display = { // 2:Setup I2C structure
        .port          = WICED_I2C_2,
        .address       = 0x3C,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .flags         = 0,
        .speed_mode    = I2C_STANDARD_SPEED_MODE,
    };

    u8g_init_wiced_i2c_device(&oled_display); // 3: initialize I2C device

    // 4: initialize communication function
    u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);

    u8g_SetFont(&u8g, u8g_font_unifont); // 5: select a font
    u8g_SetFontPosTop(&u8g); // 6: select a position

    while(1) {
        u8g_FirstPage(&u8g);

        do {
            u8g_DrawStr(&u8g, 0, 10, "Hello SCU!"); // 7: draw!
        } while (u8g_NextPage(&u8g));
    }
}

