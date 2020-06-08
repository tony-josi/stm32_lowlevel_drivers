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

  uint8_t              pin;                                 /*!< GPIO Pin @GPIO_Register_CFG */
  uint8_t              mode;                                /*!< GPIO mode  @GPIO_Register_CFG */
  uint8_t              pullup_pulldown;                     /*!< GPIO Pull-up/Pull-down @GPIO_Register_CFG */
  uint8_t              op_speed;                            /*!< GPIO Output speed @GPIO_Register_CFG */
  uint8_t              op_type;                             /*!< GPIO Output type @GPIO_Register_CFG */
  uint8_t              alternate_func_mode;                 /*!< GPIO Alternate function */

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

/* GPIOs Register CFG Macros */
/* ------------------------- */

/**
 * @addtogroup GPIO_Register_CFG
 * @brief GPIOs Mode Register CFG Macros
 *
 * @{
 *
 */
#define GPIO_MODE_IN                        (0x00000000u)   /*!< Input Floating Mode                   */
#define GPIO_MODE_OUT                       (0x00000001u)   /*!< Output Push Pull Mode                 */
#define GPIO_MODE_AF                        (0x00000002u)   /*!< Alternate Function Push Pull Mode     */
#define GPIO_MODE_ANALOG                    (0x00000003u)   /*!< Analog Mode  */
#define GPIO_MODE_IT_RT                     (0x00000004u)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define GPIO_MODE_IT_FT                     (0x00000005u)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define GPIO_MODE_IT_RFT                    (0x00000006u)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

/**
 * @brief GPIOs Output speed Register CFG Macros
 */
#define GPIO_SPEED_LOW                      (0x00000000u)  /*!< Low speed       */
#define GPIO_SPEED_MEDIUM                   (0x00000001u)  /*!< Medium speed    */
#define GPIO_SPEED_HIGH                     (0x00000002u)  /*!< High speed      */
#define GPIO_SPEED_VERY_HIGH                (0x00000003u)  /*!< Very high speed */

/**
 * @brief GPIOs Pull-up/Pull-down Register CFG Macros
 */
#define  GPIO_NOPULL                        (0x00000000u)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP                        (0x00000001u)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN                      (0x00000002u)   /*!< Pull-down activation                */

/**
 * @brief GPIOs output type register
 */
#define  GPIO_OP_PUSH_PULL                  (0x00000000u)   /*!< Push - Pull activation               */
#define  GPIO_OP_OPEN_DRAIN                 (0x00000001u)   /*!< Open Drain activation                */

/**
 * @brief GPIOs Alternate Function mode register
 */
#define  GPIO_AF_0                          (0u)            /*!< GPIO alternate function 0            */
#define  GPIO_AF_1                          (1u)            /*!< GPIO alternate function 1            */
#define  GPIO_AF_2                          (2u)            /*!< GPIO alternate function 2            */
#define  GPIO_AF_3                          (3u)            /*!< GPIO alternate function 3            */
#define  GPIO_AF_4                          (4u)            /*!< GPIO alternate function 4            */
#define  GPIO_AF_5                          (5u)            /*!< GPIO alternate function 5            */
#define  GPIO_AF_6                          (6u)            /*!< GPIO alternate function 6            */
#define  GPIO_AF_7                          (7u)            /*!< GPIO alternate function 7            */

/**
 * @brief GPIOs Pin CFG Macros
 */
#define GPIO_PIN_0                          (0u)
#define GPIO_PIN_1                          (1u)
#define GPIO_PIN_2                          (2u)
#define GPIO_PIN_3                          (3u)
#define GPIO_PIN_4                          (4u)
#define GPIO_PIN_5                          (5u)
#define GPIO_PIN_6                          (6u)
#define GPIO_PIN_7                          (7u)
#define GPIO_PIN_8                          (8u)
#define GPIO_PIN_9                          (9u)
#define GPIO_PIN_10                         (10u)
#define GPIO_PIN_11                         (11u)
#define GPIO_PIN_12                         (12u)
#define GPIO_PIN_13                         (13u)
#define GPIO_PIN_14                         (14u)
#define GPIO_PIN_15                         (15u)

/**
 * @}
 */

/**
 * @brief GPIO Port reset
 *
 */
#define GPIO_A_RESET            do { RCC->IOPRSTR |= (1 << 0); RCC->IOPRSTR &= ~(1 << 0); } while(0)
#define GPIO_B_RESET            do { RCC->IOPRSTR |= (1 << 1); RCC->IOPRSTR &= ~(1 << 1); } while(0)
#define GPIO_C_RESET            do { RCC->IOPRSTR |= (1 << 2); RCC->IOPRSTR &= ~(1 << 2); } while(0)
#define GPIO_D_RESET            do { RCC->IOPRSTR |= (1 << 3); RCC->IOPRSTR &= ~(1 << 3); } while(0)
#define GPIO_F_RESET            do { RCC->IOPRSTR |= (1 << 5); RCC->IOPRSTR &= ~(1 << 5); } while(0)

/**
 * @brief GPIO Initialize
 *
 */
Drv_Status_t LL_HAL_GPIO_Init(GPIO_Handle_t *hGPIO);

/**
 * @brief GPIO DeInitialize
 *
 */
Drv_Status_t LL_HAL_GPIO_Deinit(GPIO_RegDef_Type *pGPIOx);

/**
 * @brief GPIO Peripheral CLK Control
 *
 */
Drv_Status_t LL_HAL_GPIO_PCLK_Cntrl(GPIO_RegDef_Type *pGPIOx, uint8_t Enable);

/**
 * @brief GPIO Read input pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t *op_data);

/**
 * @brief GPIO Read input port
 *
 */
Drv_Status_t LL_HAL_GPIO_Read_IP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t *op_data);

/**
 * @brief GPIO Write output pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin, uint8_t ip_data);

/**
 * @brief GPIO Write output port
 *
 */
Drv_Status_t LL_HAL_GPIO_Write_OP_Port(GPIO_RegDef_Type *pGPIOx, uint16_t ip_data);

/**
 * @brief GPIO toggle a pin
 *
 */
Drv_Status_t LL_HAL_GPIO_Toggle_OP_Pin(GPIO_RegDef_Type *pGPIOx, uint8_t Pin);

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
