/**
 *  @file   spi_test.c
 *  @brief  Source file for GPIO test cases
 *
 *  This file contains SPI tests.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#include "stm32g070xx_ll_hal.h"
#include "driver_tests.h"

static void spi_test_gpio_init();
static void spi_test_peripheral_init();


void spi_test_init_sent_data__pol() {

  UNUSED(spi_test_gpio_init);
  UNUSED(spi_test_peripheral_init);

}

void spi_test_gpio_init() {

  /**
   * Alternate functions
   *
   * SPI2_MOSI: PB11 -> AF0
   * SPI2_NSS:  PB12 -> AF0
   * SPI2_SCK:  PB13 -> AF0
   * SPI2_MISO: PB14 -> AF0
   *
   */

  GPIO_Handle_t spi2_ph;
  spi2_ph.GPIO_regdef = GPIO_B;
  spi2_ph.GPIO_InitFields.mode = GPIO_MODE_AF;
  spi2_ph.GPIO_InitFields.op_speed = GPIO_SPEED_HIGH;
  spi2_ph.GPIO_InitFields.pullup_pulldown = GPIO_NOPULL;
  spi2_ph.GPIO_InitFields.op_type = GPIO_OP_PUSH_PULL;

  /* Initialize SPI2_MOSI */
  spi2_ph.GPIO_InitFields.alternate_func_mode = GPIO_AF_0;
  spi2_ph.GPIO_InitFields.pin = GPIO_PIN_11;
  LL_HAL_GPIO_Init(&spi2_ph);

  /* Initialize SPI2_NSS */
  spi2_ph.GPIO_InitFields.alternate_func_mode = GPIO_AF_0;
  spi2_ph.GPIO_InitFields.pin = GPIO_PIN_12;
  LL_HAL_GPIO_Init(&spi2_ph);

  /* Initialize SPI2_SCK */
  spi2_ph.GPIO_InitFields.alternate_func_mode = GPIO_AF_0;
  spi2_ph.GPIO_InitFields.pin = GPIO_PIN_13;
  LL_HAL_GPIO_Init(&spi2_ph);

  /* Initialize SPI2_MISO */
  spi2_ph.GPIO_InitFields.alternate_func_mode = GPIO_AF_0;
  spi2_ph.GPIO_InitFields.pin = GPIO_PIN_14;
  LL_HAL_GPIO_Init(&spi2_ph);


}

void spi_test_peripheral_init() {



}
