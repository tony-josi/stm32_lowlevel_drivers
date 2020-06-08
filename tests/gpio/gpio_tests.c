/**
 *  @file   gpio_test.c
 *  @brief  Source file for GPIO test cases
 *
 *  This file contains GPIO tests.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#include "stm32g070xx_ll_hal.h"
#include "driver_tests.h"

static void brute_delay() {

  for(uint32_t val = 0; val < 700000; val++);

}


void led_blink() {

  GPIO_Handle_t led;
  led.GPIO_regdef = GPIO_A;
  led.GPIO_InitFields.pin = GPIO_PIN_5;
  led.GPIO_InitFields.mode = GPIO_MODE_OUT;
  led.GPIO_InitFields.op_speed = GPIO_SPEED_HIGH;
  led.GPIO_InitFields.pullup_pulldown = GPIO_NOPULL;
  led.GPIO_InitFields.op_type = GPIO_OP_PUSH_PULL;
  LL_HAL_GPIO_PCLK_Cntrl(GPIO_A, ENABLE);
  LL_HAL_GPIO_Init(&led);

  for(uint8_t itr = 0; itr < 30; itr++) {

    LL_HAL_GPIO_Toggle_OP_Pin(GPIO_A, GPIO_PIN_5);
    brute_delay();

  }

  LL_HAL_GPIO_Deinit(GPIO_A);

}
