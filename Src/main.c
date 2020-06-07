/**
 *  @file   main.c
 *  @brief  Main file
 *
 *  This file contains the main function.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#include "stm32g0xx_device.h"
#include "stm32g070xx_gpio_lowlevel_driver.h"

int main(void)
{
	/* LL_HAL_GPIO_PCLK_Cntrl test code */
	LL_HAL_GPIO_PCLK_Cntrl(GPIO_A, ENABLE);
	LL_HAL_GPIO_PCLK_Cntrl(GPIO_F, ENABLE);


	for(;;);
}


void drv_assert_failed(uint8_t *file_name, uint32_t line_num) {

	// handle error

}
