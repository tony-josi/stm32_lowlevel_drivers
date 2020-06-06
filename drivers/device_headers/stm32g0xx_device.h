/**
 *  @file   stm32g0xx_device.h
 *  @brief  CMSIS Cortex-M0+ Device Peripheral Access Layer Header File.
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

#define FLASH_BASE_ADDR                 (0x00000000UL)	/*!< FLASH base address */
#define ROM_BASE_ADDR                   (0x1FFF0000UL)	/*!< ROM base address */
#define SRAM_BASE_ADDR                  (0x20000000UL)	/*!< SRAM base address */

/* Peripheral Bus */
#define APB1_BASE_ADDR                  (0x40000000UL)	/*!< APB1 base address */
#define APB2_BASE_ADDR                  (0x40010000UL)	/*!< APB1 base address */
#define AHB_BASE_ADDR                   (0x40020000UL)	/*!< AHB base address */
#define IOPORT_BUS_BASE_ADDR            (0x50000000UL)	/*!< IOPORT base address */

/* AHB Peripherals */
#define DMA_BASE_ADDR                   (AHB_BASE_ADDR + 0x00000000UL) /*!< DMA base address */
#define DMA_MUX_BASE_ADDR               (AHB_BASE_ADDR + 0x00000800UL) /*!< DMA MUX base address */
#define RCC_BASE_ADDR                   (AHB_BASE_ADDR + 0x00001000UL) /*!< RCC base address */
#define EXTI_BASE_ADDR                  (AHB_BASE_ADDR + 0x00001800UL) /*!< EXTI base address */
#define FLASH_AHB_BASE_ADDR             (AHB_BASE_ADDR + 0x00002000UL) /*!< FLASH base address */
#define CRC_BASE_ADDR                   (AHB_BASE_ADDR + 0x00003000UL) /*!< CRC base address */

/* APB1 Peripherals */
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

/* IOPORT_BUS Peripherals */
/* GPIOs */
#define GPIOA_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000000UL)	/*!< GPIOA base address */
#define GPIOB_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000400UL)	/*!< GPIOB base address */
#define GPIOC_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000800UL)	/*!< GPIOC base address */
#define GPIOD_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00000C00UL)	/*!< GPIOD base address */
#define GPIOF_BASE_ADDR                 (IOPORT_BUS_BASE_ADDR + 0x00001400UL)	/*!< GPIOF base address */

#endif /* __STM32G0XX_DEVICE_H */
