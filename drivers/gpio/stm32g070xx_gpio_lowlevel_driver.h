/**
 *  @file   stm32g070xx_gpio_lowlevel_driver.h
 *  @brief  GPIO Low level driver header file
 *
 *  This file contains the function declarations & structure declarations
 *  for low level GPIO functions
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#ifndef __STM32070XX_GPIO_LOWLEVEL_DRIVER_H
#define __STM32070XX_GPIO_LOWLEVEL_DRIVER_H

#include "stm32g0xx_device.h"

typedef struct {

	uint8_t pin;
	uint8_t mode;
	uint8_t pullup_pulldown;
	uint8_t op_speed;
	uint8_t alternate_func;

} GPIO_InitConfig_t;


typedef struct {

	GPIO_RegDef_Type    *GPIO_regdef;                         /*!< GPIO Register definition pointer */
	GPIO_InitConfig_t    GPIO_InitFields;                     /*!< GPIO initialization fields */

} GPIO_Handle_t;


#endif /* __STM32070XX_GPIO_LOWLEVEL_DRIVER_H */
