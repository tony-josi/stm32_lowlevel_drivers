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

static void brute_delay(uint32_t delay) {

  for(uint32_t val = 0; val < (delay * 10000); val++);

}


void led_blink__gpio__pclk__init__toggle__deinit() {

  GPIO_Handle_t led;
  led.GPIO_regdef = GPIO_A;
  led.GPIO_InitFields.pin = GPIO_PIN_5;
  led.GPIO_InitFields.mode = GPIO_MODE_OUT;
  led.GPIO_InitFields.op_speed = GPIO_SPEED_HIGH;
  led.GPIO_InitFields.pullup_pulldown = GPIO_NOPULL;
  led.GPIO_InitFields.op_type = GPIO_OP_PUSH_PULL;

  LL_HAL_GPIO_PCLK_Cntrl(GPIO_A, ENABLE);
  LL_HAL_GPIO_Init(&led);

  for(uint8_t itr = 0; itr < 300; itr++) {

    LL_HAL_GPIO_Write_OP_Port(GPIO_A, (1 << GPIO_PIN_5));
    brute_delay(10);
    LL_HAL_GPIO_Write_OP_Port(GPIO_A, (0 << GPIO_PIN_5));
    brute_delay(10);
    LL_HAL_GPIO_Write_OP_Port(GPIO_A, (1 << GPIO_PIN_5));
    brute_delay(10);
    LL_HAL_GPIO_Write_OP_Port(GPIO_A, (0 << GPIO_PIN_5));

    brute_delay(100);

  }

  LL_HAL_GPIO_Deinit(GPIO_A);

}


void led_btn_onclick_blink__gpio__pclk__init__toggle__deinit() {

  GPIO_Handle_t led, btn;
  led.GPIO_regdef = GPIO_A;
  led.GPIO_InitFields.pin = GPIO_PIN_5;
  led.GPIO_InitFields.mode = GPIO_MODE_OUT;
  led.GPIO_InitFields.op_speed = GPIO_SPEED_HIGH;
  led.GPIO_InitFields.pullup_pulldown = GPIO_NOPULL;
  led.GPIO_InitFields.op_type = GPIO_OP_PUSH_PULL;

  LL_HAL_GPIO_PCLK_Cntrl(GPIO_A, ENABLE);
  LL_HAL_GPIO_Init(&led);

  btn.GPIO_regdef = GPIO_C;
  btn.GPIO_InitFields.pin = GPIO_PIN_13;
  btn.GPIO_InitFields.mode = GPIO_MODE_IN;
  btn.GPIO_InitFields.op_speed = GPIO_SPEED_HIGH;
  btn.GPIO_InitFields.pullup_pulldown = GPIO_NOPULL;

  LL_HAL_GPIO_PCLK_Cntrl(GPIO_C, ENABLE);
  LL_HAL_GPIO_Init(&btn);

  uint8_t ip_val = 1; //prev_state = 1;

  while(1) {

    if(!LL_HAL_GPIO_Read_IP_Pin(GPIO_C, GPIO_PIN_13, &ip_val) && (ip_val == 0)) {

      ip_val = 1;
      LL_HAL_GPIO_Toggle_OP_Pin(GPIO_A, GPIO_PIN_5);
      brute_delay(20);

    }

  }

  LL_HAL_GPIO_Deinit(GPIO_A);
  LL_HAL_GPIO_Deinit(GPIO_C);

}










