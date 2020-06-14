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

void EXTI4_15_IRQHandler(void);

static void brute_delay(uint32_t delay) {

  for(uint32_t val = 0; val < (delay * 10000); val++);

}


int main(void)
{

  //led_blink__gpio__pclk__init__toggle__deinit();
  //led_btn_onclick_blink__gpio__pclk__init__toggle__deinit();
  led_btn_gpio_it_init__exti_callback();

	while(1) {

	  brute_delay(10);
	  LL_HAL_GPIO_Toggle_OP_Pin(GPIO_A, GPIO_PIN_5);

	}

	for(;;);
}


void drv_assert_failed(uint8_t *file_name, uint32_t line_num) {

	// handle error

}

void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  LL_HAL_GPIO_IRQ_Handler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}
