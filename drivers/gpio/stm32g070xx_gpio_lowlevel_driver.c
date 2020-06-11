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
#include "stm32g070xx_exti_lowlevel_driver.h"

/**
 * @brief GPIO Initialize
 *
 * @param  [in]  hGPIO   GPIO Handle
 *
 */
Drv_Status_t LL_HAL_GPIO_Init(GPIO_Handle_t *hGPIO) {

  drv_assert_param(hGPIO);
  uint32_t reg_buff = 0;

  if(hGPIO->GPIO_InitFields.mode <= GPIO_MODE_ANALOG) {

    reg_buff = hGPIO->GPIO_regdef->MODER;
    reg_buff &= ~((3U) << (hGPIO->GPIO_InitFields.pin * 2));
    reg_buff |= ((hGPIO->GPIO_InitFields.mode) << (hGPIO->GPIO_InitFields.pin * 2));
    hGPIO->GPIO_regdef->MODER = reg_buff;

    if(hGPIO->GPIO_InitFields.mode == GPIO_MODE_AF) {

      if(hGPIO->GPIO_InitFields.alternate_func_mode <= GPIO_AF_7) {

        uint8_t AFR_index, AFR_AF_bits;
        AFR_index = hGPIO->GPIO_InitFields.pin / 8;
        AFR_AF_bits = hGPIO->GPIO_InitFields.pin % 8;

        reg_buff = hGPIO->GPIO_regdef->AFR[AFR_index];
        reg_buff &= ~((0xFU) << (AFR_AF_bits * 4));
        reg_buff |= ((hGPIO->GPIO_InitFields.alternate_func_mode) << (AFR_AF_bits * 4));
        hGPIO->GPIO_regdef->AFR[AFR_index] = reg_buff;

      } else
        return DRV_ERROR;
    }

  } else if (hGPIO->GPIO_InitFields.mode <= GPIO_MODE_IT_RFT) {

    if (hGPIO->GPIO_InitFields.mode == GPIO_MODE_IT_RT) {

      EXTI->RTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
      EXTI->FTSR1 &= ~(1 << hGPIO->GPIO_InitFields.pin);

    } else if (hGPIO->GPIO_InitFields.mode == GPIO_MODE_IT_FT) {

      EXTI->FTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
      EXTI->RTSR1 &= ~(1 << hGPIO->GPIO_InitFields.pin);

    } else if (hGPIO->GPIO_InitFields.mode == GPIO_MODE_IT_RFT) {

      EXTI->FTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
      EXTI->RTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
    }

    /* Configure EXTI external interrupt selection register */
    uint8_t temp_EXTI_EXTICRx_idx = hGPIO->GPIO_InitFields.pin / EXTI_EXTICR_REG_COUNT;
    uint8_t temp_EXTI_EXTICRx_shftr = (hGPIO->GPIO_InitFields.pin % 4) * EXTI_EXTICR_EXTI_SEL_WIDTH;
    uint8_t port_code = GPIO_PORT_TO_CODE(hGPIO->GPIO_regdef);

    EXTI->EXTICR[temp_EXTI_EXTICRx_idx] = port_code << temp_EXTI_EXTICRx_shftr;

    /* Enable the Interrupt Mask Register */
    EXTI->IMR1 |= (1 << hGPIO->GPIO_InitFields.pin);

  } else {

    return DRV_ERROR;
  }

  if(hGPIO->GPIO_InitFields.op_speed <= GPIO_SPEED_VERY_HIGH) {

    reg_buff = hGPIO->GPIO_regdef->OSPEEDR;
    reg_buff &= ~((3U) << (hGPIO->GPIO_InitFields.pin * 2));
    reg_buff |= ((hGPIO->GPIO_InitFields.op_speed) << (hGPIO->GPIO_InitFields.pin * 2));
    hGPIO->GPIO_regdef->OSPEEDR = reg_buff;

  }

  if(hGPIO->GPIO_InitFields.pullup_pulldown <= GPIO_PULLDOWN) {

    reg_buff = hGPIO->GPIO_regdef->PUPDR;
    reg_buff &= ~((3U) << (hGPIO->GPIO_InitFields.pin * 2));
    reg_buff |= ((hGPIO->GPIO_InitFields.pullup_pulldown) << (hGPIO->GPIO_InitFields.pin * 2));
    hGPIO->GPIO_regdef->PUPDR = reg_buff;

  }

  if(hGPIO->GPIO_InitFields.op_type <= GPIO_OP_OPEN_DRAIN) {

    reg_buff = hGPIO->GPIO_regdef->OTYPER;
    reg_buff &= ~((1U) << (hGPIO->GPIO_InitFields.pin));
    reg_buff |= ((hGPIO->GPIO_InitFields.op_type) << (hGPIO->GPIO_InitFields.pin));
    hGPIO->GPIO_regdef->OTYPER = reg_buff;

  }

  return DRV_OK;

}

/**
 * @brief GPIO DeInitialize
 *
 */
Drv_Status_t LL_HAL_GPIO_Deinit(GPIO_RegDef_Type *pGPIOx) {

  drv_assert_param(pGPIOx);

  if(pGPIOx == GPIO_A)
    GPIO_A_RESET;
  else if(pGPIOx == GPIO_B)
    GPIO_B_RESET;
  else if(pGPIOx == GPIO_C)
    GPIO_C_RESET;
  else if(pGPIOx == GPIO_D)
    GPIO_D_RESET;
  else if(pGPIOx == GPIO_F)
    GPIO_F_RESET;
  else
    return DRV_ERROR;

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

  drv_assert_param(pGPIOx);
  uint32_t idr, mask;

  idr = pGPIOx->IDR;
  mask = (1 << Pin);
  *op_data = (uint8_t) ((idr & mask) >> Pin);

  return DRV_OK;

}


/**
 * @brief GPIO Read input port
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t *op_data) {

  drv_assert_param(pGPIOx);

  *op_data = (uint16_t) (pGPIOx->IDR & 0xFFFF);

  return DRV_OK;

}


/**
 * @brief GPIO Write output pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t ip_data) {

  drv_assert_param(pGPIOx);

  if(ip_data == SET) {

    pGPIOx->BSRR = (uint32_t)(1 << Pin);

  } else if (ip_data == RESET) {

    pGPIOx->BRR = (uint32_t)(1 << Pin);

  } else
    return DRV_ERROR;

  return DRV_OK;

}

/**
 * @brief GPIO Write output port
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t ip_data) {

  drv_assert_param(pGPIOx);

  pGPIOx->ODR = ip_data;

  return DRV_OK;

}

/**
 * @brief GPIO toggle a pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Toggle_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin) {

  drv_assert_param(pGPIOx);

  if ((pGPIOx->ODR & (1 << Pin)) != 0x00u) {

    pGPIOx->BRR = (uint32_t)(1 << Pin);

  } else {

    pGPIOx->BSRR = (uint32_t)(1 << Pin);

  }

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
