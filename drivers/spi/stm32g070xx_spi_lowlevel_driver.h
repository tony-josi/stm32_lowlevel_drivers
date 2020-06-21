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
 * @brief BIDIMODE: Bidirectional data
 * mode enable.
 *
 * This bit enables half-duplex
 * communication using common single
 * bidirectional data line.
 * Keep RXONLY bit clear when bidirectional
 * mode is active.
 *
 * @note: This bit is not used in I2S mode.
 */
#define SPI_UNIDIRECTIONAL_DATA_MODE                  (0u)                 /*!< Unidirectional data mode selected */
#define SPI_BIDIRECTIONAL_DATA_MODE                   (1u)                 /*!< Bidirectional data mode selected */


/**
 * @brief BIDIOE: Output enable in bidirectional mode
 *
 * This bit combined with the BIDIMODE bit
 * selects the direction of transfer in bidirectional mode.
 *
 * @note In master mode, the MOSI pin is used
 * and in slave mode, the MISO pin is used.
 * This bit is not used in I2S mode.
 *
 */
#define SPI_BIDIOE_OP_DISABLED                        (0u)                 /*!< Output disabled (receive-only mode */
#define SPI_BIDIOE_OP_ENABLED                         (1u)                 /*!< Output enabled (transmit-only mode */


/**
 * @brief CRCEN: Hardware CRC calculation enable
 *
 * @note This bit should be written only
 * when SPI is disabled (SPE = ‘0’) for
 * correct operation. This bit is not used in I2S mode.
 *
 */
#define SPI_CRC_CALC_DISABLED                        (0u)                  /*!< CRC calculation disabled */
#define SPI_CRC_CALC_ENABLED                         (1u)                  /*!< CRC calculation enabled */


/**
 * @brief CRCNEXT: Transmit CRC next
 *
 * @note This bit has to be written as
 * soon as the last data is written in the
 * SPIx_DR register. This bit is not used in I2S mode.
 *
 */
#define SPI_CRCNEXT_TX_BUFF                          (0u)                  /*!< Next transmit value is from TX buffer. */
#define SPI_CRCNEXT_TX_CRC_REG                       (1u)                  /*!< Next transmit value is from TX CRC register.*/


/**
 * @brief CRCL: CRC length
 *
 * This bit is set and cleared by
 * software to select the CRC length.
 *
 */
#define SPI_CRC_8_BIT                                (0u)                  /*!< 8-bit CRC length */
#define SPI_CRC_16_BIT                               (1u)                  /*!< 16-bit CRC length */


/**
 * @brief RXONLY: Receive only mode enabled.
 *
 * This bit enables simplex communication using a
 * single unidirectional line to receive data exclusively.
 * Keep BIDIMODE bit clear when receive only mode is active.
 * This bit is also useful in a multi-slave system in
 * which this particular slave is not accessed,
 * the output from the accessed slave is not corrupted.
 *
 * @note This bit is not used in I2S mode.
 *
 */
#define SPI_FULL_DUPLEX                              (0u)                 /*!< Full-duplex (Transmit and receive) */
#define SPI_HALF_DUPLEX_OUTPUT_DISABLED              (1u)                 /*!< Output disabled (Receive-only mode) */


/**
 * @brief SSM: Software slave management
 *
 * When the SSM bit is set, the NSS pin input is
 * replaced with the value from the SSI bit.
 *
 * @note This bit is not used in I2S mode and SPI TI mode.
 *
 */
#define SPI_SSM_DISABLED                              (0u)                 /*!< Software slave management disabled */
#define SPI_SSM_ENABLED                               (1u)                 /*!< Software slave management enabled */

/**
 * @brief SSI: Internal slave select
 *
 * This bit has an effect only when the SSM bit is set.
 * The value of this bit is forced onto the NSS pin and
 * the I/O value of the NSS pin is ignored.
 *
 * @note This bit is not used in I2S mode and SPI TI mode.
 *
 */
#define SPI_SSI_RESET                                 (0u)                 /*!< Internal slave select reset */
#define SPI_SSI_SET                                   (1u)                 /*!< Internal slave select set */

/**
 * @brief LSBFIRST: Frame format
 *
 *
 * @note 1. This bit should not be changed when communication is ongoing.
 *       2. This bit is not used in I2S mode and SPI TI mode.
 *
 */
#define SPI_MSB_FIRST_FRAME                           (0u)                 /*!< Data is transmitted / received with the MSB first */
#define SPI_LSB_FIRST_FRAME                           (1u)                 /*!< Data is transmitted / received with the LSB first */


/**
 * @brief SPE: SPI enable
 *
 *
 * @note When disabling the SPI, follow the procedure
 * described in Procedure for disabling the SPI on page 855 of Data sheet.
 * This bit is not used in I2S mode.
 *
 */
#define SPI_PERI_DISABLED                             (0u)                 /*!< Peripheral disabled */
#define SPI_PERI_ENABLED                              (1u)                 /*!< Peripheral enabled */


/**
 * @brief BR[2:0]: Baud rate control
 *
 *
 * @note These bits should not be changed
 * when communication is ongoing.
 * This bit is not used in I2S mode.
 *
 */

#define SPI_BR_PRESCALER_2                            (0u)                 /*!< fPCLK/2 */
#define SPI_BR_PRESCALER_4                            (1u)                 /*!< fPCLK/4 */
#define SPI_BR_PRESCALER_8                            (2u)                 /*!< fPCLK/8 */
#define SPI_BR_PRESCALER_16                           (3u)                 /*!< fPCLK/16 */
#define SPI_BR_PRESCALER_32                           (4u)                 /*!< fPCLK/32 */
#define SPI_BR_PRESCALER_64                           (5u)                 /*!< fPCLK/64 */
#define SPI_BR_PRESCALER_128                          (6u)                 /*!< fPCLK/128 */
#define SPI_BR_PRESCALER_256                          (7u)                 /*!< fPCLK/256 */


/**
 * @brief MSTR: Master selection
 *
 *
 * @note This bit should not be changed
 * when communication is ongoing.
 * This bit is not used in I2S mode.
 *
 */
#define SPI_SLAVE                                     (0u)                 /*!< Slave configuration */
#define SPI_MASTER                                    (1u)                 /*!< Master configuration */


/**
 * @brief CPOL: Clock polarity
 *
 *
 * @note This bit should not be changed
 * when communication is ongoing. This bit is not used in
 * I2S mode and SPI TI mode except the case
 * when CRC is applied at TI mode.
 *
 *
 */
#define SPI_CLK_POLARITY_0                            (0u)                 /*!< CK to 0 when idle */
#define SPI_CLK_POLARITY_1                            (1u)                 /*!< CK to 1 when idle */


/**
 * @brief CPHA: Clock phase
 *
 *
 * @note This bit should not be changed
 * when communication is ongoing. This bit is not used
 * in I2S mode and SPI TI mode except the
 * case when CRC is applied at TI mode.
 *
 */
#define SPI_CLK_PHASE_0                                 (0u)               /*!< The first clock transition
                                                                                is the first data capture edge */
#define SPI_CLK_PHASE_1                                 (1u)               /*!< The second clock transition
                                                                                is the first data capture edge */


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
