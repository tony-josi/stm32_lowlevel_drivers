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
#include "stm32g070xx_driver_def.h"

/**
 * @struct  __GPIO_InitConfig_t
 * @brief   GPIO Initialization configuration structure
 *
 */
typedef struct __GPIO_InitConfig_t {

	uint8_t pin;
	uint8_t mode;
	uint8_t pullup_pulldown;
	uint8_t op_speed;
	uint8_t alternate_func;

} GPIO_InitConfig_t;

/**
 * @struct  __GPIO_Handle_t
 * @brief   GPIO Handle
 *
 * This structure contains field to handle GPIOs
 */
typedef struct __GPIO_Handle_t {

	GPIO_RegDef_Type    *GPIO_regdef;                         /*!< GPIO Register definition pointer */
	GPIO_InitConfig_t    GPIO_InitFields;                     /*!< GPIO initialization fields */

} GPIO_Handle_t;

/**
 * @brief GPIO Initialize
 *
 */
Drv_Status_t LL_HAL_GPIO_Init(void);

/**
 * @brief GPIO DeInitialize
 *
 */
Drv_Status_t LL_HAL_GPIO_Deinit(void);

/**
 * @brief GPIO Peripheral CLK Control
 *
 */
Drv_Status_t LL_HAL_GPIO_PCLK_Cntrl(void);

/**
 * @brief GPIO Read input pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Pin(void);

/**
 * @brief GPIO Read input port
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Port(void);

/**
 * @brief GPIO Write output pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Pin(void);

/**
 * @brief GPIO Write output port
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Port(void);

/**
 * @brief GPIO toggle a pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Toggle_OP_Pin(void);

/**
 * @brief GPIO IRQ configure
 *
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Config(void);

/**
 * @brief GPIO Handle IRQ
 *
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Handling(void);



#endif /* __STM32070XX_GPIO_LOWLEVEL_DRIVER_H */
