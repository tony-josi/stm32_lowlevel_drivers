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

#define FLASH_BASE_ADDR					(0x00000000UL)	/*!< FLASH base address */
#define ROM_BASE_ADDR					(0x1FFF0000UL)	/*!< ROM base address */
#define SRAM_BASE_ADDR					(0x20000000UL)	/*!< SRAM base address */

/* Peripheral Bus */
#define APB_BASE_ADDR					(0x40000000UL)	/*!< APB base address */
#define AHB_BASE_ADDR					(0x40020000UL)	/*!< AHB base address */
#define IOPORT_BUS_BASE_ADDR			(0x50000000UL)	/*!< IOPORT base address */

/* IOPORT_BUS Peripherals */
#define GPIOA_BASE_ADDR					(IOPORT_BUS_BASE_ADDR + 0x00000000UL)	/*!< GPIOA base address */
#define GPIOB_BASE_ADDR					(IOPORT_BUS_BASE_ADDR + 0x00000400UL)	/*!< GPIOB base address */
#define GPIOC_BASE_ADDR					(IOPORT_BUS_BASE_ADDR + 0x00000800UL)	/*!< GPIOC base address */
#define GPIOD_BASE_ADDR					(IOPORT_BUS_BASE_ADDR + 0x00000C00UL)	/*!< GPIOD base address */
#define GPIOF_BASE_ADDR					(IOPORT_BUS_BASE_ADDR + 0x00001400UL)	/*!< GPIOF base address */

#endif /* __STM32G0XX_DEVICE_H */
