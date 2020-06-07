/**
 *  @file   stm32g070xx_gpio_lowlevel_driver.c
 *  @brief  GPIO Low level driver source file
 *
 *  This file contains the function definitions for low level
 *  GPIO functions
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#include "stm32g070xx_gpio_lowlevel_driver.h"

/**
 * @brief GPIO Initialize
 *
 * @param  [in]  hGPIO   GPIO Handle
 *
 */
Drv_Status_t LL_HAL_GPIO_Init(GPIO_Handle_t *hGPIO) {

	return DRV_OK;

}

/**
 * @brief GPIO DeInitialize
 *
 */
Drv_Status_t LL_HAL_GPIO_Deinit(GPIO_RegDef_Type *pGPIOx) {

	return DRV_OK;

}


/**
 * @brief GPIO Peripheral clock Initialize
 *
 * @param  [in]  pGPIOx   GPIOx Register Definition structure pointer
 * @param  [in]  Enable   Enable or Disable
 * @retval Status:
 *             - #DRV_OK        Init success
 *             - #DRV_ERROR     Init failed
 */
Drv_Status_t LL_HAL_GPIO_PCLK_Cntrl(GPIO_RegDef_Type *pGPIOx, uint8_t Enable) {

	drv_assert_param(pGPIOx);

	if(Enable == ENABLE) {

		if(pGPIOx == GPIO_A)
			GPIO_A_PCLK_EN;
		else if(pGPIOx == GPIO_B)
			GPIO_B_PCLK_EN;
		else if(pGPIOx == GPIO_C)
			GPIO_C_PCLK_EN;
		else if(pGPIOx == GPIO_D)
			GPIO_D_PCLK_EN;
		else if(pGPIOx == GPIO_F)
			GPIO_F_PCLK_EN;
		else
			return DRV_ERROR;

	} else if (Enable == DISABLE) {

		if(pGPIOx == GPIO_A)
			GPIO_A_PCLK_DI;
		else if(pGPIOx == GPIO_B)
			GPIO_B_PCLK_DI;
		else if(pGPIOx == GPIO_C)
			GPIO_C_PCLK_DI;
		else if(pGPIOx == GPIO_D)
			GPIO_D_PCLK_DI;
		else if(pGPIOx == GPIO_F)
			GPIO_F_PCLK_DI;
		else
			return DRV_ERROR;

	} else
		return DRV_ERROR;

	return DRV_OK;

}


/**
 * @brief GPIO Read input pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t *op_data) {

	return DRV_OK;

}


/**
 * @brief GPIO Read input port
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t *op_data) {

	return DRV_OK;

}


/**
 * @brief GPIO Write output pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t ip_data) {

	return DRV_OK;

}

/**
 * @brief GPIO Write output port
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t ip_data) {

	return DRV_OK;

}

/**
 * @brief GPIO toggle a pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Toggle_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin) {

	return DRV_OK;

}

/**
 * @brief GPIO IRQ configure
 *
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Config(void) {

	return DRV_OK;

}

/**
 * @brief GPIO Handle IRQ
 *
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Handling(void) {

	return DRV_OK;

}


