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

/**
 *
 *  @brief
 *
 *    N O T E S
 *  -------------
 *
 *  -------------------------------------------------
 *  | Notes on using SPI in Full duplex master mode |
 *  -------------------------------------------------
 *
 *  To enable SPI in master mode please follow the following procedure:
 *
 *  For details and examples refer to the
 *  test cases in tests/spi/spi_tests.c
 *                _____________________
 *
 *
 *  -> Initialize the GPIO's used by the SPI of interest by choosing
 *     the alternate function and other configurations
 *
 *  -> Call the LL_HAL_SPI_Init() to initialize the SPI peripheral
 *     with required configurations
 *
 *  -> If Software slave management is enabled during SPI
 *     initialization via LL_HAL_SPI_Init(), then, use LL_HAL_SPI_SSI_Cntrl()
 *     to explicitly control the NSS/CS line of the MCU to select is between
 *     master/slave
 *
 *  -> Enable the SPI peripheral by calling LL_HAL_SPI_Enable()
 *
 *  -> Use preferred mode to transmit data using the adequate functions
 *     for example, LL_HAL_SPI_Transmit()
 *
 *  -> Disable the SPI peripheral using LL_HAL_SPI_Enable()
 *
 *
 *  NOTE: Its not required to explicitly call the LL_HAL_SPI_PCLK_Cntrl() to
 *        initialize the peripheral clock as its already done inside the
 *        LL_HAL_SPI_Init().
 *
 *
 */

#include "stm32g070xx_spi_lowlevel_driver.h"

/**
 * @brief SPI Initialize
 *
 * @param  [in]  hSPI   SPI Handle
 *
 * @retval Status:
 *             - #DRV_OK        Init success
 *             - #DRV_ERROR     Init failed
 */
Drv_Status_t LL_HAL_SPI_Init(SPI_Handle_t *hSPI) {

  drv_assert_param(hSPI);
  uint32_t reg_buff = 0;

  /* Enable PCLK clock */
  LL_HAL_SPI_PCLK_Cntrl(hSPI, ENABLE);

  /* Initialize the mode of communication */
  reg_buff = hSPI->SPI_regdef->CR1;

  if(hSPI->SPI_Init.mode <= SPI_MASTER) {
    reg_buff &= ~(SPI_MODE_BIT_WIDTH << SPI_MODE_BIT_POS);
    reg_buff |= hSPI->SPI_Init.mode << SPI_MODE_BIT_POS;
  }

  /* Initialize bus configuration */
  if(hSPI->SPI_Init.bus_config == SPI_FULL_DUPLEX_BUS) {

    reg_buff &= ~(SPI_DIR_BIT_WIDTH << SPI_DIR_BIT_POS);
    reg_buff |= SPI_UNIDIRECTIONAL_DATA_MODE << SPI_DIR_BIT_POS;

  } else if(hSPI->SPI_Init.bus_config == SPI_HALF_DUPLEX_BUS) {

    reg_buff &= ~(SPI_DIR_BIT_WIDTH << SPI_DIR_BIT_POS);
    reg_buff |= SPI_BIDIRECTIONAL_DATA_MODE << SPI_DIR_BIT_POS;

  } else if(hSPI->SPI_Init.bus_config == SPI_SIMPLEX_RX_BUS) {

    reg_buff &= ~(SPI_DIR_BIT_WIDTH << SPI_DIR_BIT_POS);
    reg_buff |= SPI_UNIDIRECTIONAL_DATA_MODE << SPI_DIR_BIT_POS;

    reg_buff &= ~(SPI_RXONLY_BIT_POS << SPI_RXONLY_BIT_POS);
    reg_buff |= SPI_HALF_DUPLEX_OUTPUT_DISABLED << SPI_DIR_BIT_POS;

  } else
    return DRV_ERROR;

  /* Set baud rate for SPI */
  if(hSPI->SPI_Init.clock_speed <= SPI_BR_PRESCALER_256) {
    reg_buff &= ~(SPI_BAUD_BIT_WIDTH << SPI_BAUD_BIT_POS);
    reg_buff |= hSPI->SPI_Init.clock_speed << SPI_BAUD_BIT_POS;
  }

  /* Set clock polarity for SPI */
  if(hSPI->SPI_Init.clock_polarity <= SPI_CLK_POLARITY_1) {
    reg_buff &= ~(SPI_CLK_POL_BIT_WIDTH << SPI_CLK_POL_BIT_POS);
    reg_buff |= hSPI->SPI_Init.clock_polarity << SPI_CLK_POL_BIT_POS;
  }

  /* Set clock phase for SPI */
  if(hSPI->SPI_Init.clock_polarity <= SPI_CLK_PHASE_1) {
    reg_buff &= ~(SPI_CLK_PHA_BIT_WIDTH << SPI_CLK_PHA_BIT_POS);
    reg_buff |= hSPI->SPI_Init.clock_polarity << SPI_CLK_PHA_BIT_POS;
  }

  /* Set Software slave management for SPI */
  if(hSPI->SPI_Init.ssm <= SPI_SSM_ENABLED) {
    reg_buff &= ~(SPI_SSM_BIT_WIDTH << SPI_SSM_BIT_POS);
    reg_buff |= hSPI->SPI_Init.ssm << SPI_SSM_BIT_POS;
  }

  /* Assign buffer values to the register */
  hSPI->SPI_regdef->CR1 = reg_buff;

  reg_buff = hSPI->SPI_regdef->CR2;

  /* Set Data size: These bits configure the data length for SPI transfers */
  if((hSPI->SPI_Init.dff > SPI_DATA_SIZE_NOT_USED_3) &&
      (hSPI->SPI_Init.dff <= SPI_DATA_SIZE_16_BITS)) {
    reg_buff &= ~(SPI_DATA_SIZE_BIT_WIDTH << SPI_DATA_SIZE_BIT_POS);
    reg_buff |= hSPI->SPI_Init.dff << SPI_DATA_SIZE_BIT_POS;
  }

  /* Assign buffer values to the register */
  hSPI->SPI_regdef->CR2 = reg_buff;

  return DRV_OK;

}

/**
* @brief SPI Peripheral Enable or Disable
*
* TODO: Enable timeout for __SPI_WAIT_UNTIL_BUSY()
*
*/
Drv_Status_t LL_HAL_SPI_Enable(SPI_Handle_t *hSPI, uint8_t Enable) {

  if(Enable <= ENABLE) {

    if(Enable == DISABLE)
      __SPI_WAIT_UNTIL_BUSY(hSPI);

    uint32_t reg_buff;
    reg_buff = hSPI->SPI_regdef->CR1;

    reg_buff &= ~(SPI_PERI_BIT_WIDTH << SPI_PERI_BIT_POS);
    reg_buff |= Enable << SPI_PERI_BIT_POS;

    hSPI->SPI_regdef->CR1 = reg_buff;

    return DRV_OK;

  } else
    return DRV_ERROR;

}

/**
 * @brief SPI Peripheral CLK Control
 *
 */
Drv_Status_t LL_HAL_SPI_PCLK_Cntrl(SPI_Handle_t *pSPI, uint8_t Enable) {

  drv_assert_param(pSPI);

  /* Enable peripheral clock for the given SPI */
  if(Enable == ENABLE) {

    if(pSPI->SPI_regdef == SPI1)
      SPI1_PCLK_EN;
    else if(pSPI->SPI_regdef == SPI2)
      SPI2_PCLK_EN;
    else
      return DRV_ERROR;


    /* Disable peripheral clock for the given SPI */
  } else if (Enable == DISABLE) {

    if(pSPI->SPI_regdef == SPI1)
      SPI1_PCLK_DI;
    else if(pSPI->SPI_regdef == SPI2)
      SPI2_PCLK_DI;
    else
      return DRV_ERROR;

  } else

    /* Invalid Enable option */
    return DRV_ERROR;

  return DRV_OK;

}

/**
 * @brief SPI Internal slave select
 *
 */
Drv_Status_t LL_HAL_SPI_SSI_Cntrl(SPI_Handle_t *pSPI, uint8_t Enable) {

  drv_assert_param(pSPI);

  if(Enable <= ENABLE) {
    uint32_t reg_buff = 0;
    reg_buff = pSPI->SPI_regdef->CR1;
    reg_buff &= ~(SPI_SSI_BIT_WIDTH << SPI_SSI_BIT_POS);
    reg_buff |= (Enable << SPI_SSI_BIT_POS);
    pSPI->SPI_regdef->CR1 = reg_buff;
    return DRV_OK;
  }

  return DRV_ERROR;

}


/**
 * @brief SPI SS output enable
 *
 */
Drv_Status_t LL_HAL_SPI_SSOE_Cntrl(SPI_Handle_t *pSPI, uint8_t Enable) {

  drv_assert_param(pSPI);

  if(Enable <= ENABLE) {
    uint32_t reg_buff = 0;
    reg_buff = pSPI->SPI_regdef->CR2;
    reg_buff &= ~(SPI_SSOE_BIT_WIDTH << SPI_SSOE_BIT_POS);
    reg_buff |= (Enable << SPI_SSOE_BIT_POS);
    pSPI->SPI_regdef->CR2 = reg_buff;
    return DRV_OK;
  }

  return DRV_ERROR;

}


/**
 * @brief SPI Transmit in blocking mode
 *
 */
Drv_Status_t LL_HAL_SPI_Transmit(SPI_Handle_t *pSPI, uint8_t *pTX_buff, uint32_t len) {

  drv_assert_param(pSPI);

  /*
   * TODO: Add timeout
   * TODO: Handle other data size or dff
   * */

  if(len > 0) {

    while(len > 0) {

      while(SPI_GET_FLAG_STATUS(pSPI->SPI_regdef->SR, (SPI_TXE_BIT_POS << SPI_TXE_BIT_WIDTH)) == SPI_RESET);

      if(pSPI->SPI_Init.dff == SPI_DATA_SIZE_8_BITS) {

        pSPI->SPI_regdef->DR = (uint8_t) *pTX_buff;
        --len;
        ++pTX_buff;

      } else if(pSPI->SPI_Init.dff == SPI_DATA_SIZE_16_BITS) {

        pSPI->SPI_regdef->DR = (uint16_t) *((uint16_t *) pTX_buff);
        len -= 2;
        pTX_buff += 2;

      } else
        /* Only supports 8bit & 16bit DS */
        return DRV_ERROR;
    }

    return DRV_OK;

  }

  return DRV_ERROR;

}


/**
 * @brief SPI Receive in blocking mode
 *
 */
Drv_Status_t LL_HAL_SPI_Receive(SPI_Handle_t *pSPI, uint8_t *pTX_buff, uint32_t len) {

  drv_assert_param(pSPI);

  /*
   * TODO: Add timeout
   * TODO: Handle other data size or dff
   * */

  if(len > 0) {

    while(len > 0) {

      while(SPI_GET_FLAG_STATUS(pSPI->SPI_regdef->SR, (SPI_RXNXE_BIT_POS << SPI_RXNXE_BIT_WIDTH)) == SPI_RESET);

      if(pSPI->SPI_Init.dff == SPI_DATA_SIZE_8_BITS) {

        *pTX_buff = (uint8_t) pSPI->SPI_regdef->DR;
        --len;
        ++pTX_buff;

      } else if(pSPI->SPI_Init.dff == SPI_DATA_SIZE_16_BITS) {

        *((uint16_t *) pTX_buff) = (uint16_t) pSPI->SPI_regdef->DR;
        len -= 2;
        pTX_buff += 2;

      } else
        /* Only supports 8bit & 16bit DS */
        return DRV_ERROR;
    }

    return DRV_OK;

  }

  return DRV_ERROR;

}



/**
 * @brief SPI DeInitialize
 *
 */
Drv_Status_t LL_HAL_SPI_Deinit(SPI_Handle_t *pSPI) {

  drv_assert_param(pSPI);

  __SPI_WAIT_UNTIL_BUSY(pSPI);

  /* Check for corresponding SPI and reset
   * using appropriate macros */
  if(pSPI->SPI_regdef == SPI1)
    SPI1_RESET;
  else if(pSPI->SPI_regdef == SPI2)
    SPI2_RESET;
  else
    return DRV_ERROR;

  return DRV_OK;

}


