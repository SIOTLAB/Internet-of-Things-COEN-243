//------------------------------------------------------------
// SCU's Internet of Things Research Lab (SIOTLAB)
// Santa Clara University (SCU)
// Santa Clara, California
//------------------------------------------------------------

// This application is based on the Cypress WICED platform

//------------------------------------------------------------


/**
 * Features demonstrated
 *  - WICED scan API
 *
 * This application snippet regularly scans for nearby Wi-Fi access points
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   Each time the application scans, a list of Wi-Fi access points in
 *   range is printed to the UART
 *
 */

#include <stdlib.h>
#include "wiced.h"

#define DELAY_BETWEEN_SCANS  5000

typedef struct
{
    wiced_semaphore_t   semaphore;      // Semaphore used for signaling scan complete
    uint32_t            result_count;   // Count to measure the total scan results
} app_scan_data_t;


wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );


void application_start( )
{
    wiced_init( );

    while(1)
    {
        wiced_time_t    scan_start_time;
        wiced_time_t    scan_end_time;
        app_scan_data_t scan_data;

        /* Initialize the semaphore that will tell us when the scan is complete */
        wiced_rtos_init_semaphore(&scan_data.semaphore);
        scan_data.result_count = 0;
        WPRINT_APP_INFO( ( "Waiting for scan results...\n" ) );
        WPRINT_APP_INFO( ("  # Type\tBSSID\tRSSI\tRate\tChan\tSecurity\tSSID\n" ) );
        WPRINT_APP_INFO( ("-----------------------------------------------------------------\n" ) );

        /* Start the scan */
        wiced_time_get_time(&scan_start_time);
        wiced_wifi_scan_networks(scan_result_handler, &scan_data );

        /* Wait until scan is complete */
        wiced_rtos_get_semaphore(&scan_data.semaphore, WICED_WAIT_FOREVER);
        wiced_time_get_time(&scan_end_time);

        WPRINT_APP_INFO( ("\nScan complete in %lu milliseconds\n", (unsigned long )(scan_end_time - scan_start_time) ) );

        /* Clean up */
        wiced_rtos_deinit_semaphore(&scan_data.semaphore);

        /* Issuing next scan after some delay (optional) */
        wiced_rtos_delay_milliseconds(DELAY_BETWEEN_SCANS);
    }
}

/*
 * Callback function to handle scan results
 */
wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    /* Validate the input arguments */
    wiced_assert("Bad args", malloced_scan_result != NULL);

    if ( malloced_scan_result != NULL )
    {
        app_scan_data_t* scan_data  = (app_scan_data_t*)malloced_scan_result->user_data;

        malloc_transfer_to_curr_thread( malloced_scan_result );

        if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE )
        {
            wiced_scan_result_t* record = &malloced_scan_result->ap_details;

            WPRINT_APP_INFO( ( "%3ld ", scan_data->result_count ) );
            print_scan_result(record);

            scan_data->result_count++;
        }
        else
        {
            wiced_rtos_set_semaphore( &scan_data->semaphore );
        }

        free( malloced_scan_result );
    }
    return WICED_SUCCESS;
}
