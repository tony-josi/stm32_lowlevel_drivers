/**
 *  @file   stm32g070xx_spi_lowlevel_driver.c
 *  @brief  Serial Peripheral Interface
 *          Low level driver header file
 *
 *  This file contains the function declarations & structure declarations
 *  for low level SPI functions. The Serial Peripheral Interface is a synchronous
 *  serial communication interface specification used for
 *  short-distance communication, primarily in embedded systems.
 *
 *  @author         Tony Josi   https://tonyjosi97.github.io/profile/
 *  @copyright      Copyright (C) 2020 Tony Josi
 *  @bug            No known bugs.
 */

#ifndef __STM32070XX_SPI_LOWLEVEL_DRIVER_H
#define __STM32070XX_SPI_LOWLEVEL_DRIVER_H

#include "stm32g0xx_device.h"
#include "stm32g070xx_driver_def.h"

/**
 * @struct  __SPI_InitConfig_t
 * @brief   SPI Initialization configuration structure
 *
 */
typedef struct __SPI_InitConfig_t {

  uint8_t              mode;                              /*!< SPI mode   */
  uint8_t              bus_config;                        /*!< SPI bus configuration  */
  uint8_t              clock_speed;                       /*!< SPI clock frequency  */
  uint8_t              dff;                               /*!< SPI DFF  */
  uint8_t              clock_polarity;                    /*!< SPI Clock Polarity */
  uint8_t              clock_phase;                       /*!< SPI Clock Phase */
  uint8_t              ssm;                               /*!< SPI Software slave management */

} SPI_InitConfig_t;

/**
 * @struct  __SPI_Handle_t
 * @brief   SPI Handle
 *
 * This structure contains field to handle SPIs
 */
typedef struct __SPI_Handle_t {

  SPI_RegDef_Type    *SPI_regdef;                         /*!< SPI Register definition pointer */

} SPI_Handle_t;


/**
 * @brief SPI Controller reset
 *
 */
#define APBRSTR2_SPI1_RESET                               12
#define APBRSTR1_SPI2_RESET                               14


/**
 * @brief SPI Controller reset
 *
 */
#define SPI1_RESET            RCC->APBRSTR2 |= (1 << APBRSTR2_SPI1_RESET)    /*!< SPI1 Reset */
#define SPI2_RESET            RCC->APBRSTR1 |= (1 << APBRSTR1_SPI2_RESET)    /*!< SPI2 Reset */

/**
 * @brief SPI Initialize
 *
 */
Drv_Status_t LL_HAL_SPI_Init(SPI_Handle_t *hSPI, SPI_InitConfig_t);

/**
 * @brief SPI DeInitialize
 *
 */
Drv_Status_t LL_HAL_SPI_Deinit(SPI_RegDef_Type *pSPI);

/**
 * @brief SPI Peripheral CLK Control
 *
 */
Drv_Status_t LL_HAL_SPI_PCLK_Cntrl(SPI_RegDef_Type *pSPI, uint8_t Enable);

/**
 * @brief SPI Transmit in blocking mode
 *
 */
Drv_Status_t LL_HAL_SPI_Transmit(SPI_Handle_t *pSPI, uint8_t *pTX_buff, uint32_t len);

/**
 * @brief SPI Transmit in interrupt mode
 *
 */
Drv_Status_t LL_HAL_SPI_Transmit_IT(SPI_Handle_t *pSPI, uint8_t *pTX_buff, uint32_t len);

/**
 * @brief SPI Receive in blocking mode
 *
 */
Drv_Status_t LL_HAL_SPI_Recieve(SPI_Handle_t *pSPI, uint8_t *pRX_buff, uint32_t len);

/**
 * @brief SPI Receive in interrupt mode
 *
 */
Drv_Status_t LL_HAL_SPI_Recieve_IT(SPI_Handle_t *pSPI, uint8_t *pRX_buff, uint32_t len);

/**
 * @brief SPI IRQ Interrupt configure
 *
 */
Drv_Status_t LL_HAL_SPI_IRQ_Interupt_Config(uint8_t IRQ_Num, uint8_t Enable);

/**
 * @brief SPI IRQ Priority configure
 *
 */
Drv_Status_t LL_HAL_SPI_IRQ_Priority_Config(uint8_t IRQ_Num, uint8_t Priority);

/**
 * @brief SPI Handle IRQ
 *
 */
Drv_Status_t LL_HAL_SPI_IRQ_Handler(SPI_Handle_t *pSPI);


#endif /* __STM32070XX_SPI_LOWLEVEL_DRIVER_H */
