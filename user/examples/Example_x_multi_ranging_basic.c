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
#include "vl53l8cx_api.h"

int example_dual(VL53L8CX_Configuration *p_dev1, VL53L8CX_Configuration *p_dev2)
{

	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, i;
	uint8_t					isReady1, isReady2;
	VL53L8CX_ResultsData 	Results1, Results2;		/* Results data from VL53L8CX */


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	status = vl53l8cx_is_alive(p_dev1, &isAlive);
	if(!isAlive || status)
	{
		printf("First VL53L8CX not detected at requested address\n");
		return status;
	}

	status = vl53l8cx_is_alive(p_dev2, &isAlive);
	if(!isAlive || status)
	{
		printf("Second VL53L8CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(p_dev1);
	if(status)
	{
		printf("First VL53L8CX ULD Loading failed\n");
		return status;
	}

	VL53L8CX_WaitMs(&p_dev1->platform, 2);

	status = vl53l8cx_init(p_dev2);
	if(status)
	{
		printf("Second VL53L8CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L8CX ULD ready ! (Version : %s)\n",
			VL53L8CX_API_REVISION);

/*
	status = vl53l8cx_set_resolution(p_dev1, VL53L8CX_RESOLUTION_4X4);
    status |= vl53l8cx_set_ranging_mode(p_dev1, VL53L8CX_RANGING_MODE_CONTINUOUS);
    status |= vl53l8cx_set_integration_time_ms(p_dev1, 100);
    status |= vl53l8cx_set_ranging_frequency_hz(p_dev1, 5);

	status = vl53l8cx_set_resolution(p_dev2, VL53L8CX_RESOLUTION_4X4);
    status |= vl53l8cx_set_ranging_mode(p_dev2, VL53L8CX_RANGING_MODE_CONTINUOUS);
    status |= vl53l8cx_set_integration_time_ms(p_dev2, 100);
    status |= vl53l8cx_set_ranging_frequency_hz(p_dev2, 5);
*/

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l8cx_start_ranging(p_dev1);
	VL53L8CX_WaitMs(&p_dev1->platform, 2);
	status = vl53l8cx_start_ranging(p_dev2);

	loop = 0;
	while(loop < 20)
	{
		/* Use polling function to know when a new measurement is ready */
 
		status = vl53l8cx_check_data_ready(p_dev1, &isReady1);

		status = vl53l8cx_check_data_ready(p_dev2, &isReady2);

		if(isReady1)
		{
			vl53l8cx_get_ranging_data(p_dev1, &Results1);
		}

		if(isReady2)
		{
			vl53l8cx_get_ranging_data(p_dev2, &Results2);
		}
		
		if (isReady1)
		{
			/* As the sensor is set in 4x4 mode by default, we have a total 
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			printf("satel 1 Print data no : %3u \n", p_dev1->streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results1.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
					Results1.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}

		if (isReady2)
		{
			/* As the sensor is set in 4x4 mode by default, we have a total 
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			printf("satel 2 Print data no : %3u \n", p_dev2->streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results2.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
					Results2.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform file, not in API) */
		VL53L8CX_WaitMs(&p_dev1->platform, 2);
	}
	status = vl53l8cx_stop_ranging(p_dev1);
	status = vl53l8cx_stop_ranging(p_dev2);
	printf("End of ULD demo\n");
	return status;
}

int example_multi(VL53L8CX_Configuration tdev[], uint8_t max_dev)
{
	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, i, idev;
	uint8_t					isReady;
	VL53L8CX_ResultsData 	Results;


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/
	status = 0;
	for (idev = 0; idev < max_dev; idev++) {
		
		status = vl53l8cx_is_alive(&tdev[idev], &isAlive);
		if(!isAlive || status)
		{
			printf("VL53L8CX #%d not detected at requested address \n",idev);
			return status;
		}
		status = vl53l8cx_init(&tdev[idev]);
		if(status)
		{
			printf("VL53L8CX #%d ULD Loading failed\n", idev);
			return status;
		}

		status = vl53l8cx_set_resolution(&tdev[idev], VL53L8CX_RESOLUTION_4X4);
		status |= vl53l8cx_set_ranging_mode(&tdev[idev], VL53L8CX_RANGING_MODE_CONTINUOUS);
		status |= vl53l8cx_set_integration_time_ms(&tdev[idev], 10);
		status |= vl53l8cx_set_ranging_frequency_hz(&tdev[idev], 1);

		if(!status)
			printf("VL53L8CX #%d initialized\n",idev);

	}

	printf("VL53L8CX ULD ready (Version : %s)\n",
			VL53L8CX_API_REVISION);

	for (idev = 0; idev < max_dev; idev++) {
		status = vl53l8cx_start_ranging(&tdev[idev]);
		if(!status)
			printf("VL53L8CX #%d started\n",idev);
	}

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	loop = 0;
	while(loop < (10 * max_dev))
	{
		/* Use polling function to know when a new measurement is ready */
 
		for (idev = 0; idev < max_dev; idev++) {
		
			status = vl53l8cx_check_data_ready(&tdev[idev], &isReady);

			if(isReady)
			{
				vl53l8cx_get_ranging_data(&tdev[idev], &Results);
			}
			
			if (isReady)
			{
				/* As the sensor is set in 4x4 mode by default, we have a total 
				 * of 16 zones to print. For this example, only the data of first zone are 
				 * print */
				printf("satel #%d Print data no : %3u \n", idev, (&tdev[idev])->streamcount);
				for(i = 0; i < 16; i++)
				{
					printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
						i,
						Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
						Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
				}
				printf("\n");
				loop++;
			}
			VL53L8CX_WaitMs(NULL, 2);
		}		
	}

	for (idev = 0; idev < max_dev; idev++)
		status = vl53l8cx_stop_ranging(&tdev[idev]);

	printf("End of ULD demo\n");
	return status;
}
