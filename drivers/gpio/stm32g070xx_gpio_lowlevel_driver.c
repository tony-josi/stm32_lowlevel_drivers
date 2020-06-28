/**
 *  @file   stm32g070xx_gpio_lowlevel_driver.c
 *  @brief  GPIO Low level driver source file
 *
 *  This file contains the function definitions for low level
 *  GPIO functions. A general-purpose input/output is an
 *  uncommitted digital signal pin on an integrated
 *  circuit or electronic circuit board whose behavior—including
 *  whether it acts as input or output—is controllable by
 *  the user at run time.
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
 * @retval Status:
 *             - #DRV_OK        Init success
 *             - #DRV_ERROR     Init failed
 */
Drv_Status_t LL_HAL_GPIO_Init(GPIO_Handle_t *hGPIO) {

  drv_assert_param(hGPIO);
  uint32_t reg_buff = 0;

  /* Enable the clock of the corresponding port */
  LL_HAL_GPIO_PCLK_Cntrl(hGPIO->GPIO_regdef, ENABLE);

  /* Check if the mode is valid */
  if (hGPIO->GPIO_InitFields.mode <= GPIO_MODE_IT_RFT) {

    reg_buff = hGPIO->GPIO_regdef->MODER;
    reg_buff &= ~((3U) << (hGPIO->GPIO_InitFields.pin * 2));

    /* Assign input mode for all interrupt configurations */
    if(hGPIO->GPIO_InitFields.mode < GPIO_MODE_IT_RT)
      reg_buff |= ((hGPIO->GPIO_InitFields.mode) << (hGPIO->GPIO_InitFields.pin * 2));

    /* Update MODER register of the corresponding GPIO Port */
    hGPIO->GPIO_regdef->MODER = reg_buff;

    /* If mode is either Input mode, General purpose output mode, or
     * Alternate function mode */
    if(hGPIO->GPIO_InitFields.mode <= GPIO_MODE_ANALOG) {

      /* If alternate function mode */
      if(hGPIO->GPIO_InitFields.mode == GPIO_MODE_AF) {

        /* Check for valid alternate function */
        if(hGPIO->GPIO_InitFields.alternate_func_mode <= GPIO_AF_7) {

          uint8_t AFR_index, AFR_AF_bits;
          AFR_index = hGPIO->GPIO_InitFields.pin / 8;
          AFR_AF_bits = hGPIO->GPIO_InitFields.pin % 8;

          reg_buff = hGPIO->GPIO_regdef->AFR[AFR_index];
          reg_buff &= ~((0xFU) << (AFR_AF_bits * 4));
          reg_buff |= ((hGPIO->GPIO_InitFields.alternate_func_mode) << (AFR_AF_bits * 4));

          /* Set the AFR register with preferred AF */
          hGPIO->GPIO_regdef->AFR[AFR_index] = reg_buff;

        } else

          /* Return error for invalid parameters */
          return DRV_ERROR;
      }

    } else {

      /* Set the RTSR1 register for rising edge trigger */
      if (hGPIO->GPIO_InitFields.mode == GPIO_MODE_IT_RT) {

        EXTI->RTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
        EXTI->FTSR1 &= ~(1 << hGPIO->GPIO_InitFields.pin);

        /* Set FTR1 for falling edge */
      } else if (hGPIO->GPIO_InitFields.mode == GPIO_MODE_IT_FT) {

        EXTI->FTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
        EXTI->RTSR1 &= ~(1 << hGPIO->GPIO_InitFields.pin);

        /* Set both RTSR1 & FTSR1 for both rising and falling trigger */
      } else if (hGPIO->GPIO_InitFields.mode == GPIO_MODE_IT_RFT) {

        EXTI->FTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
        EXTI->RTSR1 |= (1 << hGPIO->GPIO_InitFields.pin);
      }

      /* Configure EXTI external interrupt selection register */
      uint8_t temp_EXTI_EXTICRx_idx = hGPIO->GPIO_InitFields.pin / EXTI_EXTICR_REG_COUNT;
      uint8_t temp_EXTI_EXTICRx_shftr = (hGPIO->GPIO_InitFields.pin % 4) * EXTI_EXTICR_EXTI_SEL_WIDTH;
      uint8_t port_code = GPIO_PORT_TO_CODE(hGPIO->GPIO_regdef);

      EXTI->EXTICR[temp_EXTI_EXTICRx_idx] |= port_code << temp_EXTI_EXTICRx_shftr;

      /* Enable the Interrupt Mask Register */
      EXTI->IMR1 |= (1 << hGPIO->GPIO_InitFields.pin);
    }

  } else
    return DRV_ERROR;

  /* Set the output speed of the Pin */
  if(hGPIO->GPIO_InitFields.op_speed <= GPIO_SPEED_VERY_HIGH) {

    reg_buff = hGPIO->GPIO_regdef->OSPEEDR;
    reg_buff &= ~((3U) << (hGPIO->GPIO_InitFields.pin * 2));
    reg_buff |= ((hGPIO->GPIO_InitFields.op_speed) << (hGPIO->GPIO_InitFields.pin * 2));
    hGPIO->GPIO_regdef->OSPEEDR = reg_buff;

  }

  /* Set the Push/Pull configuration of the pin */
  if(hGPIO->GPIO_InitFields.pullup_pulldown <= GPIO_PULLDOWN) {

    reg_buff = hGPIO->GPIO_regdef->PUPDR;
    reg_buff &= ~((3U) << (hGPIO->GPIO_InitFields.pin * 2));
    reg_buff |= ((hGPIO->GPIO_InitFields.pullup_pulldown) << (hGPIO->GPIO_InitFields.pin * 2));
    hGPIO->GPIO_regdef->PUPDR = reg_buff;

  }

  /* Set out put type of this pin */
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
 * @param  [in]  hGPIO   GPIO Handle
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_Deinit(GPIO_RegDef_Type *pGPIOx) {

  drv_assert_param(pGPIOx);

  /* Check for corresponding port and reset
   * using appropriate macros */
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
 *
 * @param  [in]  Enable   Enable or Disable
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_PCLK_Cntrl(GPIO_RegDef_Type *pGPIOx, uint8_t Enable) {

  drv_assert_param(pGPIOx);

  /* Enable peripheral clock for the given port */
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


    /* Disable peripheral clock for the given port */
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

    /* Invalid Enable option */
    return DRV_ERROR;

  return DRV_OK;

}


/**
 * @brief GPIO Read input pin
 *
 * @param  [in]  pGPIOx   GPIOx Register Definition structure pointer
 *
 * @param  [in]  Pin      GPIO pin
 *
 * @param  [out] op_data  Output data pointer
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t *op_data) {

  drv_assert_param(pGPIOx);
  uint32_t idr, mask;

  idr = pGPIOx->IDR;

  /* Mask for IDR given pin */
  mask = (1 << Pin);

  /* Read the corresponding pin from
   * IDR register of given GPIO port */
  *op_data = (uint8_t) ((idr & mask) >> Pin);

  return DRV_OK;

}


/**
 * @brief GPIO Read input port
 *
 * @param  [in]  pGPIOx   GPIOx Register Definition structure pointer
 *
 * @param  [out] op_data  Output data pointer
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t *op_data) {

  drv_assert_param(pGPIOx);

  /* Read first 16 bits of IDR
   * register for port reading */
  *op_data = (uint16_t) (pGPIOx->IDR & 0xFFFF);

  return DRV_OK;

}


/**
 * @brief GPIO Write output pin
 *
 * @param  [in]  pGPIOx   GPIOx Register Definition structure pointer
 *
 * @param  [in]  Pin      GPIO pin
 *
 * @param  [in]  ip_data  Input data
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t ip_data) {

  drv_assert_param(pGPIOx);

  /* To set the pin */
  if(ip_data == SET) {

    /* Set the GPIO port bit set/reset
     * register bit of given pin */
    pGPIOx->BSRR = (uint32_t)(1 << Pin);

    /* To reset the pin */
  } else if (ip_data == RESET) {

    /* Set the GPIO port bit reset
     * register bit of given pin */
    pGPIOx->BRR = (uint32_t)(1 << Pin);

  } else
    return DRV_ERROR;

  return DRV_OK;

}

/**
 * @brief GPIO Write output port
 *
 * @param  [in]  pGPIOx   GPIOx Register Definition structure pointer
 *
 * @param  [in]  ip_data  Input data
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t ip_data) {

  drv_assert_param(pGPIOx);

  /* Set the ODR register with
   * given data to write to OP port*/
  pGPIOx->ODR = ip_data;

  return DRV_OK;

}

/**
 * @brief GPIO toggle a pin
 *
 * @param  [in]  pGPIOx   GPIOx Register Definition structure pointer
 *
 * @param  [in]  Pin      GPIO pin
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_Toggle_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin) {

  drv_assert_param(pGPIOx);

  /* Check for previous pin bit field, if set */
  if ((pGPIOx->ODR & (1 << Pin)) != 0x00u) {

    /* Set the GPIO port bit reset
     * register bit of given pin */
    pGPIOx->BRR = (uint32_t)(1 << Pin);

    /* If not set */
  } else {

    /* Set the GPIO port bit set/reset
     * register bit of given pin */
    pGPIOx->BSRR = (uint32_t)(1 << Pin);

  }

  return DRV_OK;

}

/**
 * @brief GPIO IRQ Interrupt configure
 *
 * @param  [in]  IRQ_Num   IRQ number of the corresponding
 *                         EXTI line/lines.
 *
 * @param  [in]  Enable    Enable or Disable pin
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Interupt_Config(uint8_t IRQ_Num, uint8_t Enable) {

  /* Bit alignment and address are based on
   * ARMv6-M Architecture Reference Manual */

  if(IRQ_Num <= 31) {

    if(Enable == ENABLE) {

      /* Set the given IRQ by setting
       * ISER register of NVIC */
      NVIC->ISER[0] = (uint32_t)(1UL << (((uint32_t)IRQ_Num) & 0x1FUL));

      /* To disable given IRQ */
    } else if(Enable == DISABLE) {

      /* Reset the given IRQ by setting
       * ICER register of NVIC */
      NVIC->ICER[0] = (uint32_t)(1UL << (((uint32_t)IRQ_Num) & 0x1FUL));

    } else
      return DRV_ERROR;

  } else
    return DRV_ERROR;

  return DRV_OK;

}

/**
 * @brief GPIO IRQ Priority configure
 *
 * @param  [in]  IRQ_Num   IRQ number of the corresponding
 *                         EXTI line/lines.
 *
 * @param  [in]  Priority  Priority value, should be between [0,3]
 *
 * @retval Status:
 *             - #DRV_OK         success
 *             - #DRV_ERROR      failed
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Priority_Config(uint8_t IRQ_Num, uint8_t Priority) {

 /* Check if IRQ number and Priority are valid */
  if ((IRQ_Num < 32) && (Priority < 4)) {

    uint8_t reg_indx = IRQ_Num / NVIC_IPR_IRQ_PER_REG;
    uint8_t reg_shftr = (IRQ_Num % NVIC_IPR_IRQ_PER_REG) * NVIC_IPR_BIT_WIDTH;

    /* Bit alignment and address are based on
     * ARMv6-M Architecture Reference Manual */

    /* Set the corresponding IP register */
    NVIC->IP[reg_indx] |= ((Priority << NVIC_IPR_PER_IRQ_SHFT) << reg_shftr);

  } else
    return DRV_ERROR;

  return DRV_OK;

}

/**
 * @brief GPIO Handle IRQ
 *
 * @param  [in]  Pin      GPIO pin
 *
 * @retval Status:
 *             - #DRV_OK        Success
 */
Drv_Status_t LL_HAL_GPIO_IRQ_Handler(uint16_t Pin) {

  /* NOTE: Use SYSCFG interrupt line X
   * status register (SYSCFG_ITLINEX) to collect all pending
   * interrupt sources associated with each interrupt
   * line into a single register.
   *
   * This allows users to check by single read which
   * peripheral requires service in case more than
   * one source is associated to the interrupt line.
   *
   * All bits in those registers are read only,
   * set by hardware when there is corresponding interrupt
   * request pending and cleared by resetting the interrupt
   * source flags in the peripheral registers. */

  /* Check if the rising pending register of
   * given pin is activated */
  if (__HAL_GPIO_EXTI_RISING_IT_STATUS((1 << Pin)) != 0x00u) {

    /* Clear the pending register */
    __HAL_GPIO_EXTI_RISING_IT_CLEAR(1 << Pin);  /* Each bit is cleared by writing 1 into it. */

    /* Call the interrupt call back routine */
    LL_HAL_GPIO_EXTI_Rising_Callback(Pin);
  }

  /* Check if the falling pending register of
   * given pin is activated */
  if (__HAL_GPIO_EXTI_FALLING_IT_STATUS((1 << Pin)) != 0x00u) {

    /* Clear the pending register */
    __HAL_GPIO_EXTI_FALLING_IT_CLEAR(1 << Pin);  /* Each bit is cleared by writing 1 into it. */

    /* Call the interrupt call back routine */
    LL_HAL_GPIO_EXTI_Falling_Callback(Pin);
  }

  return DRV_OK;

}

/**
  * @brief  EXTI line rising edge detection callback.
  *
  * @param  [in]  GPIO_Pin  Specifies the port pin
  *                         connected to corresponding EXTI line.
  *
  * @retval None
  */
__WEAK void LL_HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

  UNUSED(GPIO_Pin);

  /* NOTE: DIFINED AS __WEAK, IMPLEMENT THIS
   * FUNCTION => void LL_HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
   * IN USER CODE, AVOID MODIFICATION HERE.
   */
}

/**
  * @brief  EXTI line falling edge detection callback.
  *
  * @param  [in]  GPIO_Pin  Specifies the port pin
  *                         connected to corresponding EXTI line.
  *
  * @retval None
  */
__WEAK void LL_HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {

  UNUSED(GPIO_Pin);

  /* NOTE: DIFINED AS __WEAK, IMPLEMENT THIS
   * FUNCTION => void LL_HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin);
   * IN USER CODE, AVOID MODIFICATION HERE.
   */

}
