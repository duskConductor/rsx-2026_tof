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
/*  VL53L8CX ULD multiple targets  */
/***********************************/
/*
* This example shows the possibility of VL53L8CX to get/set params. It
* initializes the VL53L8CX ULD, set a configuration, and starts
* a ranging to capture 10 frames.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l8cx_api.h"

int example5(VL53L8CX_Configuration *p_dev)
{
	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i, j;
	VL53L8CX_ResultsData 	Results;		/* Results data from VL53L8CX */

	
	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	status = vl53l8cx_is_alive(p_dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L8CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(p_dev);
	if(status)
	{
		printf("VL53L8CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L8CX ULD ready ! (Version : %s)\n",
			VL53L8CX_API_REVISION);
			

	/*********************************/
	/*	Set nb target per zone       */
	/*********************************/

	/* Each zone can output between 1 and 4 targets. By default the output
	 * is set to 1 targets, but user can change it using macro
	 * VL53L8CX_NB_TARGET_PER_ZONE located in file 'platform.h'.
	 */

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
 
		isReady = VL53L8CX_wait_for_dataready(&p_dev->platform);

		if(isReady)
		{
			vl53l8cx_get_ranging_data(p_dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print */
			printf("Print data no : %3u\n", p_dev->streamcount);
			for(i = 0; i < 16; i++)
			{
				/* Print per zone results. These results are the same for all targets */
				printf("Zone %3u : %2u, %6d, %6d, ",
					i,
					Results.nb_target_detected[i],
					(int)Results.ambient_per_spad[i],
					(int)Results.nb_spads_enabled[i]);

				for(j = 0; j < VL53L8CX_NB_TARGET_PER_ZONE; j++)
				{
					/* Print per target results. These results depends of the target nb */
					uint16_t idx = VL53L8CX_NB_TARGET_PER_ZONE * i + j;
					printf("Target[%1u] : %2u, %4d, %6d, %3u, ",
						j,
						Results.target_status[idx],
						Results.distance_mm[idx],
						(int)Results.signal_per_spad[idx],
						Results.range_sigma_mm[idx]);
				}
				printf("\n");
			}
			printf("\n");
			loop++;
		}
	}

	status = vl53l8cx_stop_ranging(p_dev);
	printf("End of ULD demo\n");
	return status;
}
