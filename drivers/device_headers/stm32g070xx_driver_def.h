
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

#define ENABLE						(1U)
#define DISABLE						(0U)
#define SET						    (ENABLE)
#define RESET						(DISABLE)


/**
  * @brief  Driver Status structures definition
  */
typedef enum {

  DRV_OK       = 0x00U,
  DRV_ERROR    = 0x01U,
  DRV_BUSY     = 0x02U,
  DRV_TIMEOUT  = 0x03U

} Drv_Status_t;


#ifdef  DRV_ASSERT_PARAMS_CHECK
/**
  * @brief  The drv_assert_param macro is used for functions parameters check.
  */
#define drv_assert_param(expr) ((expr) ? (void)0U : drv_assert_failed((uint8_t *)__FILE__, __LINE__))

void drv_assert_failed(uint8_t *file, uint32_t line);

#else

#define drv_assert_param(expr) ((void)0U)

#endif /* DRV_ASSERT_PARAMS_CHECK */





#endif /* __STM32G0XX_DRIVER_DEF_H */




