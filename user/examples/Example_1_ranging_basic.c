/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/***********************************/
/*   VL53L8CX ULD basic example    */
/***********************************/
/*
* This example is the most basic. It initializes the VL53L8CX ULD, and starts
* a ranging to capture 10 frames.
*
* By default, ULD is configured to have the following settings :
* - Resolution 4x4
* - Ranging period 1Hz
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int example1(VL53L8CX_Configuration *p_dev)
{

	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady;
	int i;
	VL53L8CX_ResultsData 	Results;		/* Results data from VL53L8CX */


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	status = vl53l8cx_is_alive(p_dev, &isAlive);
	if(status || !isAlive)
	{
		printf("VL53L8CX not detected at requested address\n");
		return status;
	}


	// Initialize (Load 84kb firmware)
	status = vl53l8cx_init(p_dev);
    if(status)
    {
        printf("VL53L8CX ULD Loading failed. Error code: %u\n", status);
        return status;
    }

	printf("Firmware loaded; setting resolution\n");


	status = vl53l8cx_set_resolution(p_dev, VL53L8CX_RESOLUTION_8X8);
	status |= vl53l8cx_set_ranging_frequency_hz(p_dev, 15);

	if(status)
	{
		printf("VL53L8CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L8CX ULD ready ! 8x8 mode! (Version : %s)\n",
			VL53L8CX_API_REVISION);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l8cx_start_ranging(p_dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */
		isReady = 0;
		int timeout = 0;
		// isReady = VL53L8CX_wait_for_dataready(&p_dev->platform);
		// isReady = wait_for_dataready(&p_dev->platform);
		
		while (!isReady && timeout < 100) {
			status = vl53l8cx_check_data_ready(p_dev, &isReady);
			usleep(1000);
			timeout++;
		}

		if(isReady)
		{
			status = vl53l8cx_get_ranging_data(p_dev, &Results);

            printf("Frame: %3u\n", p_dev->streamcount);
            
            // Print 8x8 grid (all 64 zones)
            for(i = 0; i < 64; i++) 
            {
                // Multi-target index: VL53L8CX_NB_TARGET_PER_ZONE is typically 1 by default
                int target_idx = VL53L8CX_NB_TARGET_PER_ZONE * i;
                
                printf("Z%2d: %4dmm (Stat:%u) | ", 
                    i, 
                    Results.distance_mm[target_idx],
                    Results.target_status[target_idx]);
                
                if ((i + 1) % 8 == 0) printf("\n"); // New line every 8 zones
            }
            printf("--------------------------------------------------\n");
            loop++;
		}
	}

	status = vl53l8cx_stop_ranging(p_dev);
	printf("End of ULD demo\n");
	return status;
}
