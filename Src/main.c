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

  /* GPIO Test Case Calls */
  /* -------------------- */

  //gpio_test_led_blink__gpio__pclk__init__toggle__deinit();
  //gpio_test_led_btn_onclick_blink__gpio__pclk__init__toggle__deinit();
  //gpio_test_led_btn_gpio_it_init__exti_callback();

  /* SPI Test Case Calls */
  /* ------------------- */

  //void spi_test_init_sent_data__pol();

	while(1) {

	}

	for(;;);
}


void drv_assert_failed(uint8_t *file_name, uint32_t line_num) {

	// handle error

}


