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

#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>

#include <stdio.h>
#include <string.h>

#include "vl53l8cx_api.h"

#include "examples.h"

int exit_main_loop = 0;

void sighandler(int signal)
{
	printf("SIGNAL Handler called, signal = %d\n", signal);
	exit_main_loop  = 1;
}

int main(int argc, char ** argv)
{
	#define NB_OF_DEV 2
	uint8_t i;
	char choice[20];
	int status;
	VL53L8CX_Configuration 	Dev[NB_OF_DEV];

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	memset(&Dev, 0, sizeof(Dev));
	Dev[0].platform.spi_num = 0;
	Dev[0].platform.spi_cs = 0;

	Dev[1].platform.spi_num = 0;
	Dev[1].platform.spi_cs = 1;

	//Dev[2].platform.spi_num = 1;
	//Dev[2].platform.spi_cs = 0;

	//Dev[3].platform.spi_num = 1;
	//Dev[3].platform.spi_cs = 2;

	for (i=0; i< NB_OF_DEV; i++)
	{
		status = vl53l8cx_comms_init(&Dev[i].platform);
		if(status)
		{
			printf("VL53L8CX comms init failed on spidev%d.%d\n", Dev[i].platform.spi_num, Dev[i].platform.spi_cs);
			return -1;
		}
	}

	printf("Starting dual ranging with ULD version %s\n", VL53L8CX_API_REVISION);

	do {
		printf("----------------------------------------------------------------------------------------------------------\n");
		printf(" VL53L8CX uld driver test dual devices example menu \n");
		printf(" ------------------ Ranging menu ------------------\n");
		printf(" 1 : first device #0 only ranging \n");
		printf(" 2 : second device #1 only ranging\n");
		printf(" 3 : third device #2 only ranging\n");
		printf(" 4 : fourth device #3 only ranging\n");
		printf(" 5 : both devices ranging\n");
		printf(" 6 : exit\n");
		printf("----------------------------------------------------------------------------------------------------------\n");

		printf("Your choice ?\n ");
		scanf("%s", choice);

		if (strcmp(choice, "1") == 0) {
			printf("Starting Test device #0\n");
			status = example1(&Dev[0]);
			printf("\n");
		}
		else if (strcmp(choice, "2") == 0) {
			printf("Starting Test device #1\n");
			status = example1(&Dev[1]);
			printf("\n");
		}
//		else if (strcmp(choice, "3") == 0) {
//			printf("Starting Test device #2\n");
//			status = example1(&Dev[2]);
//			printf("\n");
//		}
//		else if (strcmp(choice, "4") == 0) {
//			printf("Starting Test device #3\n");
//			status = example1(&Dev[3]);
//			printf("\n");
//		}
		else if (strcmp(choice, "5") == 0) {
			printf("Starting Test 5\n");
			status = example_multi(Dev, NB_OF_DEV);
			printf("\n");
		}
		
		else if (strcmp(choice, "6") == 0){
			exit_main_loop = 1;
		}
		
		else{
			printf("Invalid choice\n");
		}

	} while (!exit_main_loop);

	for (i=0; i< NB_OF_DEV; i++)
	{
		vl53l8cx_comms_close(&Dev[i].platform);
	}

	return 0;
}
