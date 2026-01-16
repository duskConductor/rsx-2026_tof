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

#ifndef EXAMPLES_H_
#define EXAMPLES_H_

int example1(VL53L8CX_Configuration *p_dev);
int example2(VL53L8CX_Configuration *p_dev);
int example3(VL53L8CX_Configuration *p_dev);
int example4(VL53L8CX_Configuration *p_dev);
int example5(VL53L8CX_Configuration *p_dev);
int example6(VL53L8CX_Configuration *p_dev);
int example7(VL53L8CX_Configuration *p_dev);
int example8(VL53L8CX_Configuration *p_dev);
int example9(VL53L8CX_Configuration *p_dev);
int example10(VL53L8CX_Configuration *p_dev);
int example11(VL53L8CX_Configuration *p_dev);
int example_dual(VL53L8CX_Configuration *p_dev1, VL53L8CX_Configuration *p_dev2);
int example_multi(VL53L8CX_Configuration tdev[], uint8_t max_dev);

#endif

