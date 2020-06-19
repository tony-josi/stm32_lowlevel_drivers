/**
 *  @file   stm32g070xx_spi_lowlevel_driver.c
 *  @brief  Serial Peripheral Interface
 *          Low level driver source file
 *
 *  This file contains the function definitions for low level
 *  SPI functions. The Serial Peripheral Interface is a synchronous
 *  serial communication interface specification used for
 *  short-distance communication, primarily in embedded systems.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#include "stm32g070xx_spi_lowlevel_driver.h"

/**
 * @brief SPI Peripheral CLK Control
 *
 */
Drv_Status_t LL_HAL_SPI_PCLK_Cntrl(SPI_RegDef_Type *pSPI, uint8_t Enable) {

  drv_assert_param(pGPIOx);

  /* Enable peripheral clock for the given SPI */
  if(Enable == ENABLE) {

    if(pSPI == SPI1)
      SPI1_PCLK_EN;
    else if(pSPI == SPI2)
      SPI2_PCLK_EN;
    else
      return DRV_ERROR;


    /* Disable peripheral clock for the given SPI */
  } else if (Enable == DISABLE) {

    if(pSPI == SPI1)
      SPI1_PCLK_DI;
    else if(pSPI == SPI2)
      SPI2_PCLK_DI;
    else
      return DRV_ERROR;

  } else

    /* Invalid Enable option */
    return DRV_ERROR;

  return DRV_OK;

}
