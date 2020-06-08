/**
 *  @file   stm32g0xx_device.h
 *  @brief  Cortex-M0+ Device Peripheral Access Layer Header File.
 *
 *  This file contains all the peripheral register's definitions, bits
 *  definitions and memory mapping for stm32g070xx devices.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#ifndef __STM32G0XX_DEVICE_H
#define __STM32G0XX_DEVICE_H

#include <stdint.h>
#include "stm32g0xx_device_types.h"

/* Core memory address */
/* ------------------- */

#define FLASH_BASE_ADDR                 (0x08000000UL)  /*!< FLASH base address */
#define ROM_BASE_ADDR                   (0x1FFF0000UL)  /*!< ROM base address */
#define SRAM_BASE_ADDR                  (0x20000000UL)  /*!< SRAM base address */

/* Peripheral Bus */
/* -------------- */

#define APB1_BASE_ADDR                  (0x40000000UL)  /*!< APB1 base address */
#define APB2_BASE_ADDR                  (0x40010000UL)  /*!< APB1 base address */
#define AHB_BASE_ADDR                   (0x40020000UL)  /*!< AHB base address */
#define IOPORT_BUS_BASE_ADDR            (0x50000000UL)  /*!< IOPORT base address */

/* AHB Peripherals */
/* --------------- */

#define DMA_BASE_ADDR                   (AHB_BASE_ADDR + 0x00000000UL) /*!< DMA base address */
#define DMA_MUX_BASE_ADDR               (AHB_BASE_ADDR + 0x00000800UL) /*!< DMA MUX base address */
#define RCC_BASE_ADDR                   (AHB_BASE_ADDR + 0x00001000UL) /*!< RCC base address */
#define EXTI_BASE_ADDR                  (AHB_BASE_ADDR + 0x00001800UL) /*!< EXTI base address */
#define FLASH_AHB_BASE_ADDR             (AHB_BASE_ADDR + 0x00002000UL) /*!< FLASH base address */
#define CRC_BASE_ADDR                   (AHB_BASE_ADDR + 0x00003000UL) /*!< CRC base address */

/* APB1 Peripherals */
/* ---------------- */

#define TIM3_BASE_ADDR                  (APB1_BASE_ADDR + 0x00000400UL) /*!< TIM3 base address */
#define TIM6_MUX_BASE_ADDR              (APB1_BASE_ADDR + 0x00001000UL) /*!< TIM6 MUX base address */
#define TIM7_BASE_ADDR                  (APB1_BASE_ADDR + 0x00001400UL) /*!< TIM7 base address */
#define TIM14_BASE_ADDR                 (APB1_BASE_ADDR + 0x00002000UL) /*!< TIM14 base address */
#define RTC_AHB_BASE_ADDR               (APB1_BASE_ADDR + 0x00002800UL) /*!< RTC base address */
#define WWDG_BASE_ADDR                  (APB1_BASE_ADDR + 0x00002C00UL) /*!< WWDG base address */
#define IWDG_BASE_ADDR                  (APB1_BASE_ADDR + 0x00003000UL) /*!< IWDG base address */
#define SPI2_BASE_ADDR                  (APB1_BASE_ADDR + 0x00003800UL) /*!< SPI2 base address */
#define USART2_BASE_ADDR                (APB1_BASE_ADDR + 0x00004400UL) /*!< USART2 base address */
#define USART3_BASE_ADDR                (APB1_BASE_ADDR + 0x00004800UL) /*!< USART3 base address */
#define USART4_BASE_ADDR                (APB1_BASE_ADDR + 0x00004C00UL) /*!< USART4 base address */
#define I2C1_BASE_ADDR                  (APB1_BASE_ADDR + 0x00005400UL) /*!< I2C1 base address */
#define I2C2_BASE_ADDR                  (APB1_BASE_ADDR + 0x00005800UL) /*!< I2C2 MUX base address */
#define PWR_BASE_ADDR                   (APB1_BASE_ADDR + 0x00007000UL) /*!< PWR base address */
#define TAMP_BASE_ADDR                  (APB1_BASE_ADDR + 0x0000B000UL) /*!< TAMP base address */

/* APB2 Peripherals */
/* ---------------- */

#define SYSCFG_BASE_ADDR                (APB2_BASE_ADDR + 0x00000000UL) /*!< SYSCFG base address */
#define SYSCFG_ITLINE_BASE_ADDR         (APB2_BASE_ADDR + 0x00000080UL) /*!< SYSCFG ITLINE base address */
#define ADC_BASE_ADDR                   (APB2_BASE_ADDR + 0x00002400UL) /*!< ADC base address */
#define TIM1_BASE_ADDR                  (APB2_BASE_ADDR + 0x00002C00UL) /*!< TIM1 base address */
#define SPI1_I2S1_BASE_ADDR             (APB2_BASE_ADDR + 0x00003000UL) /*!< SPI1 base address */
#define USART1_BASE_ADDR                (APB2_BASE_ADDR + 0x00003800UL) /*!< USART1 MUX base address */
#define TIM15_BASE_ADDR                 (APB2_BASE_ADDR + 0x00004000UL) /*!< TIM15 base address */
#define TIM16_MUX_BASE_ADDR             (APB2_BASE_ADDR + 0x00004400UL) /*!< TIM16 MUX base address */
#define TIM17_BASE_ADDR                 (APB2_BASE_ADDR + 0x00004800UL) /*!< TIM17 base address */
#define DBG_BASE_ADDR                   (APB2_BASE_ADDR + 0x00005800UL) /*!< DBG base address */

/* IOPORT_BUS Peripherals, GPIOs */
/* ----------------------------- */

#define GPIOA_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000000UL) /*!< GPIOA base address */
#define GPIOB_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000400UL) /*!< GPIOB base address */
#define GPIOC_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000800UL) /*!< GPIOC base address */
#define GPIOD_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000C00UL) /*!< GPIOD base address */
#define GPIOF_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00001400UL) /*!< GPIOF base address */


/* Peripheral register structures */
/* ------------------------------ */

/**
  * @brief General Purpose Input Output
  */
typedef struct {

  __VL uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __VL uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __VL uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __VL uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __VL uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __VL uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __VL uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __VL uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __VL uint32_t AFR[2];      /*!< GPIO alternate function registers, AFRL, AFRH
                                  AFR[0] -> (Alternate function low register), Address offset: 0x20
                                  AFR[1] -> (Alternate function high register),
                                  Address offset: 0x24                                             */
  __VL uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */

} GPIO_RegDef_Type;

/**
  * @brief Reset and Clock Control
  */
typedef struct {

  __VL uint32_t CR;          /*!< Clock Sources Control Register,                                     Address offset: 0x00 */
  __VL uint32_t ICSCR;       /*!< Internal Clock Sources Calibration Register,                        Address offset: 0x04 */
  __VL uint32_t CFGR;        /*!< Regulated Domain Clocks Configuration Register,                     Address offset: 0x08 */
  __VL uint32_t PLLCFGR;     /*!< System PLL configuration Register,                                  Address offset: 0x0C */
       uint32_t RSRVD0;      /*!< Reserved,                                                           Address offset: 0x10 */
       uint32_t RSRVD1;      /*!< Reserved,                                                           Address offset: 0x14 */
  __VL uint32_t CIER;        /*!< Clock Interrupt Enable Register,                                    Address offset: 0x18 */
  __VL uint32_t CIFR;        /*!< Clock Interrupt Flag Register,                                      Address offset: 0x1C */
  __VL uint32_t CICR;        /*!< Clock Interrupt Clear Register,                                     Address offset: 0x20 */
  __VL uint32_t IOPRSTR;     /*!< IO port reset register,                                             Address offset: 0x24 */
  __VL uint32_t AHBRSTR;     /*!< AHB peripherals reset register,                                     Address offset: 0x28 */
  __VL uint32_t APBRSTR1;    /*!< APB peripherals reset register 1,                                   Address offset: 0x2C */
  __VL uint32_t APBRSTR2;    /*!< APB peripherals reset register 2,                                   Address offset: 0x30 */
  __VL uint32_t IOPENR;      /*!< IO port enable register,                                            Address offset: 0x34 */
  __VL uint32_t AHBENR;      /*!< AHB peripherals clock enable register,                              Address offset: 0x38 */
  __VL uint32_t APBENR1;     /*!< APB peripherals clock enable register1,                             Address offset: 0x3C */
  __VL uint32_t APBENR2;     /*!< APB peripherals clock enable register2,                             Address offset: 0x40 */
  __VL uint32_t IOPSMENR;    /*!< IO port clocks enable in sleep mode register,                       Address offset: 0x44 */
  __VL uint32_t AHBSMENR;    /*!< AHB peripheral clocks enable in sleep mode register,                Address offset: 0x48 */
  __VL uint32_t APBSMENR1;   /*!< APB peripheral clocks enable in sleep mode register1,               Address offset: 0x4C */
  __VL uint32_t APBSMENR2;   /*!< APB peripheral clocks enable in sleep mode register2,               Address offset: 0x50 */
  __VL uint32_t CCIPR;       /*!< Peripherals Independent Clocks Configuration Register,              Address offset: 0x54 */
       uint32_t RSRVD2;      /*!< Reserved,                                                           Address offset: 0x58 */
  __VL uint32_t BDCR;        /*!< Backup Domain Control Register,                                     Address offset: 0x5C */
  __VL uint32_t CSR;         /*!< Unregulated Domain Clock Control and Status Register,               Address offset: 0x60 */

} RCC_RegDef_Type;


/* GPIOs pointed to respective register locations */
/* ---------------------------------------------- */

#define GPIO_A                        ((GPIO_RegDef_Type *) GPIOA_BASE_ADDR)   /*!< GPIOA #GPIO_RegDef_Type pointer */
#define GPIO_B                        ((GPIO_RegDef_Type *) GPIOB_BASE_ADDR)   /*!< GPIOB #GPIO_RegDef_Type pointer */
#define GPIO_C                        ((GPIO_RegDef_Type *) GPIOC_BASE_ADDR)   /*!< GPIOC #GPIO_RegDef_Type pointer */
#define GPIO_D                        ((GPIO_RegDef_Type *) GPIOD_BASE_ADDR)   /*!< GPIOD #GPIO_RegDef_Type pointer */
#define GPIO_F                        ((GPIO_RegDef_Type *) GPIOF_BASE_ADDR)   /*!< GPIOF #GPIO_RegDef_Type pointer */

/* RCC macro pointed to respective register locations */
/* -------------------------------------------------- */

#define RCC                           ((RCC_RegDef_Type *) RCC_BASE_ADDR)      /*!< RCC #RCC_RegDef_Type pointer */


/* GPIOs Clock enable macros */
/* ------------------------- */

/**
 * @addtogroup Peripheral_Clock_Enable
 * @brief GPIO Peripheral Clock enable
 */
#define GPIO_A_PCLK_EN               ((RCC->IOPENR) |= (1 << 0))               /*!< GPIO A Peripheral Clock enable */
#define GPIO_B_PCLK_EN               ((RCC->IOPENR) |= (1 << 1))               /*!< GPIO B Peripheral Clock enable */
#define GPIO_C_PCLK_EN               ((RCC->IOPENR) |= (1 << 2))               /*!< GPIO C Peripheral Clock enable */
#define GPIO_D_PCLK_EN               ((RCC->IOPENR) |= (1 << 3))               /*!< GPIO D Peripheral Clock enable */
#define GPIO_F_PCLK_EN               ((RCC->IOPENR) |= (1 << 5))               /*!< GPIO F Peripheral Clock enable */

/**
 * @addtogroup Peripheral_Clock_Enable
 * @brief I2C Peripheral Clock enable
 */
#define I2C1_PCLK_EN                 ((RCC->APBENR1) |= (1 << 21))             /*!< I2C1 Peripheral Clock enable */
#define I2C2_PCLK_EN                 ((RCC->APBENR1) |= (1 << 22))             /*!< I2C2 Peripheral Clock enable */

/**
 * @addtogroup Peripheral_Clock_Enable
 * @brief USART Peripheral Clock enable
 */
#define USART1_PCLK_EN               ((RCC->APBENR2) |= (1 << 14))             /*!< USART1 Peripheral Clock enable */
#define USART2_PCLK_EN               ((RCC->APBENR1) |= (1 << 17))             /*!< USART2 Peripheral Clock enable */
#define USART3_PCLK_EN               ((RCC->APBENR1) |= (1 << 18))             /*!< USART3 Peripheral Clock enable */
#define USART4_PCLK_EN               ((RCC->APBENR1) |= (1 << 19))             /*!< USART4 Peripheral Clock enable */

/**
 * @addtogroup Peripheral_Clock_Enable
 * @brief SPI Peripheral Clock enable
 */
#define SPI1_PCLK_EN                 ((RCC->APBENR2) |= (1 << 12))             /*!< SPI1 Peripheral Clock enable */
#define SPI2_PCLK_EN                 ((RCC->APBENR1) |= (1 << 14))             /*!< SPI2 Peripheral Clock enable */

/**
 * @addtogroup Peripheral_Clock_Enable
 * @brief SYSCFG Peripheral Clock enable
 */
#define SYSCFG_PCLK_EN               ((RCC->APBENR2) |= (1 << 0))              /*!< SYSCFG Peripheral Clock enable */


/* GPIOs Clock disable macros */
/* ------------------------- */

/**
 * @addtogroup Peripheral_Clock_Disable
 * @brief GPIO Peripheral Clock disable
 */
#define GPIO_A_PCLK_DI               ((RCC->IOPENR) &= ~(1 << 0))              /*!< GPIO A Peripheral Clock enable */
#define GPIO_B_PCLK_DI               ((RCC->IOPENR) &= ~(1 << 1))              /*!< GPIO B Peripheral Clock enable */
#define GPIO_C_PCLK_DI               ((RCC->IOPENR) &= ~(1 << 2))              /*!< GPIO C Peripheral Clock disable */
#define GPIO_D_PCLK_DI               ((RCC->IOPENR) &= ~(1 << 3))              /*!< GPIO D Peripheral Clock disable */
#define GPIO_F_PCLK_DI               ((RCC->IOPENR) &= ~(1 << 5))              /*!< GPIO F Peripheral Clock disable */

/**
 * @addtogroup Peripheral_Clock_Disable
 * @brief I2C Peripheral Clock disable
 */
#define I2C1_PCLK_DI                 ((RCC->APBENR1) &= ~(1 << 21))            /*!< I2C1 Peripheral Clock disable */
#define I2C2_PCLK_DI                 ((RCC->APBENR1) &= ~(1 << 22))            /*!< I2C2 Peripheral Clock disable */

/**
 * @addtogroup Peripheral_Clock_Disable
 * @brief USART Peripheral Clock disable
 */
#define USART1_PCLK_DI               ((RCC->APBENR2) &= ~(1 << 14))            /*!< USART1 Peripheral Clock disable */
#define USART2_PCLK_DI               ((RCC->APBENR1) &= ~(1 << 17))            /*!< USART2 Peripheral Clock disable */
#define USART3_PCLK_DI               ((RCC->APBENR1) &= ~(1 << 18))            /*!< USART3 Peripheral Clock disable */
#define USART4_PCLK_DI               ((RCC->APBENR1) &= ~(1 << 19))            /*!< USART4 Peripheral Clock disable */

/**
 * @addtogroup Peripheral_Clock_Disable
 * @brief SPI Peripheral Clock disable
 */
#define SPI1_PCLK_DI                 ((RCC->APBENR2) &= ~(1 << 12))            /*!< SPI1 Peripheral Clock disable */
#define SPI2_PCLK_DI                 ((RCC->APBENR1) &= ~(1 << 14))            /*!< SPI2 Peripheral Clock disable */

/**
 * @addtogroup Peripheral_Clock_Disable
 * @brief SYSCFG Peripheral Clock disable
 */
#define SYSCFG_PCLK_DI               ((RCC->APBENR2) &= ~(1 << 0))             /*!< SYSCFG Peripheral Clock disable */






















#endif /* __STM32G0XX_DEVICE_H */
