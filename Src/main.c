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

#include "driver_tests.h"
#include "stm32g070xx_ll_hal.h"

int main(void)
{

  //led_blink__gpio__pclk__init__toggle__deinit();
  led_btn_onclick_blink__gpio__pclk__init__toggle__deinit();
  //gpio_it_init__gpio_init();

	for(;;);
}


void drv_assert_failed(uint8_t *file_name, uint32_t line_num) {

	// handle error

}
