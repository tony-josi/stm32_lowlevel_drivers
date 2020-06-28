/**
 *  @file   driver_tests.c
 *  @brief  Driver test header file
 *
 *  This file contains headers for the test functions for each driver
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */


#ifndef __DRIVER_TESTS_H
#define __DRIVER_TESTS_H

/* GPIO Test Case Declarations */
/* --------------------------- */

void gpio_test_led_blink__gpio__pclk__init__toggle__deinit();
void gpio_test_led_btn_onclick_blink__gpio__pclk__init__toggle__deinit();
void gpio_test_led_btn_gpio_it_init__exti_callback();


/* SPI Test Case Declarations */
/* -------------------------- */

void spi_test_init_sent_data__pol();

#endif /* __DRIVER_TESTS_H */
