
/**
 *  @file   stm32g0xx_driver_def.h
 *  @brief  Core Driver Definitions
 *
 *  This file contains common defines, enumeration, macros and
 *  structures definitions.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#ifndef __STM32G0XX_DRIVER_DEF_H
#define __STM32G0XX_DRIVER_DEF_H

/**
  * @brief  Driver Status structures definition
  */
typedef enum {

  DRV_OK       = 0x00U,
  DRV_ERROR    = 0x01U,
  DRV_BUSY     = 0x02U,
  DRV_TIMEOUT  = 0x03U

} Drv_Status_t;


#endif /* __STM32G0XX_DRIVER_DEF_H */




