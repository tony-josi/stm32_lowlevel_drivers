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

void led_blink__gpio__pclk__init__toggle__deinit();
void led_btn_onclick_blink__gpio__pclk__init__toggle__deinit();
void gpio_it_init__gpio_init();


#endif /* __DRIVER_TESTS_H */
