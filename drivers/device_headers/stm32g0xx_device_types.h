/**
 *  @file   stm32g0xx_device_types.h
 *  @brief  IO types definitions
 *
 *  This file contains macro definitions for custom IO types used
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#ifndef __STM32G0XX_DEVICE_TYPES_H
#define __STM32G0XX_DEVICE_TYPES_H

#define __VL                              volatile /*!< Volatile type macro */

#if  defined ( __GNUC__ )
#ifndef __WEAK
#define __WEAK   __attribute__((weak))
#endif /* __WEAK */
#endif /* __GNUC__ */

#define UNUSED(X) (void)X                          /* To avoid gcc/g++ warnings */

#endif /* __STM32G0XX_DEVICE_TYPES_H */
