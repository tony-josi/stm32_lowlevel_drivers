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
  SPI_InitConfig_t    SPI_Init;                           /*!< SPI Initialization configuration structure */

} SPI_Handle_t;


/**
 * @brief SPI set and reset macros
 *
 */
#define SPI_SET                                       (1u)
#define SPI_RESET                                     (0u)

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
 * @brief SPI Mode - bit info
 *
 */
#define SPI_MODE_BIT_POS                              (2u)                 /*!< SPI mode bit position */
#define SPI_MODE_BIT_WIDTH                            (1u)                 /*!< SPI mode bit width */


/**
 * @brief SPI Bus Configuration
 *
 */
#define SPI_FULL_DUPLEX_BUS                           (0u)                 /*!< Full duplex communication */
#define SPI_HALF_DUPLEX_BUS                           (1u)                 /*!< Half duplex communication */
#define SPI_SIMPLEX_TX_BUS                            (2u)                 /*!< Simplex transmit only communication */
#define SPI_SIMPLEX_RX_BUS                            (3u)                 /*!< Simplex receive only  communication */


/**
 * @brief SPI Mode - bit info
 *
 */
#define SPI_DIR_BIT_POS                               (15u)                /*!< SPI Bidirectional data mode enable bit position */
#define SPI_DIR_BIT_WIDTH                             (1u)                 /*!< SPI Bidirectional data mode enable bit width */

/**
 * @brief SPI Controller reset
 *
 */
#define APBRSTR2_SPI1_RESET                           12
#define APBRSTR1_SPI2_RESET                           14


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
#define SPI_UNIDIRECTIONAL_DATA_MODE                 (0u)                 /*!< Unidirectional data mode selected */
#define SPI_BIDIRECTIONAL_DATA_MODE                  (1u)                 /*!< Bidirectional data mode selected */


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
#define SPI_BIDIOE_OP_DISABLED                       (0u)                 /*!< Output disabled (receive-only mode */
#define SPI_BIDIOE_OP_ENABLED                        (1u)                 /*!< Output enabled (transmit-only mode */


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
 * @brief SPI RXONLY - bit info
 *
 */
#define SPI_RXONLY_BIT_POS                           (10u)                /*!< SPI mode bit position */
#define SPI_RXONLY_BIT_WIDTH                         (1u)                 /*!< SPI mode bit width */


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
 * @brief SPI Software slave management bit info
 *
 */
#define SPI_SSM_BIT_POS                               (9u)                /*!< SPI Software slave management bit position */
#define SPI_SSM_BIT_WIDTH                             (1u)                 /*!< SPI Software slave management bit width */

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
 * @brief SPI Software slave management bit info
 *
 */
#define SPI_SSM_BIT_POS                               (9u)                /*!< SPI Software slave management bit position */
#define SPI_SSM_BIT_WIDTH                             (1u)                 /*!< SPI Software slave management bit width */


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
 * @brief SPI enable bit info
 *
 */
#define SPI_PERI_BIT_POS                              (6u)                /*!< SPI Baud rate bit position */
#define SPI_PERI_BIT_WIDTH                            (1u)                /*!< SPI Baud rate bit width */


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
 * @brief SPI Baud rate bit info
 *
 */
#define SPI_BAUD_BIT_POS                              (3u)                /*!< SPI Baud rate bit position */
#define SPI_BAUD_BIT_WIDTH                            (7u)                /*!< SPI Baud rate bit width */


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
 * @brief SPI Clock polarity - bit info
 *
 */
#define SPI_CLK_POL_BIT_POS                           (1u)                /*!< SPI Clock polarity bit position */
#define SPI_CLK_POL_BIT_WIDTH                         (1u)                /*!< SPI Clock polarity bit width */


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
#define SPI_CLK_PHASE_0                               (0u)                /*!< The first clock transition
                                                                                is the first data capture edge */
#define SPI_CLK_PHASE_1                               (1u)                /*!< The second clock transition
                                                                                is the first data capture edge */

/**
 * @brief SPI Clock phase - bit info
 *
 */
#define SPI_CLK_PHA_BIT_POS                           (0u)                /*!< SPI Clock phase bit position */
#define SPI_CLK_PHA_BIT_WIDTH                         (1u)                /*!< SPI Clock phase bit width */


/**
 * @brief DS[3:0]: Data size
 *
 * These bits configure the data length for SPI transfers.
 * If software attempts to write one of the
 * “Not used” values, they are forced to the value “0111” (8-bit)
 *
 * Note: This bit is not used in I2S mode.
 *
 */
#define SPI_DATA_SIZE_NOT_USED_1                      (0u)               /*!< Not used */
#define SPI_DATA_SIZE_NOT_USED_2                      (1u)               /*!< Not used */
#define SPI_DATA_SIZE_NOT_USED_3                      (2u)               /*!< Not used */
#define SPI_DATA_SIZE_4_BITS                          (3u)               /*!< 4 bit DS */
#define SPI_DATA_SIZE_5_BITS                          (4u)               /*!< 5 bit DS */
#define SPI_DATA_SIZE_6_BITS                          (5u)               /*!< 6 bit DS */
#define SPI_DATA_SIZE_7_BITS                          (6u)               /*!< 7 bit DS */
#define SPI_DATA_SIZE_8_BITS                          (7u)               /*!< 8 bit DS */
#define SPI_DATA_SIZE_9_BITS                          (8u)               /*!< 9 bit DS */
#define SPI_DATA_SIZE_10_BITS                         (9u)               /*!< 10 bit DS */
#define SPI_DATA_SIZE_11_BITS                         (10u)              /*!< 11 bit DS */
#define SPI_DATA_SIZE_12_BITS                         (11u)              /*!< 12 bit DS */
#define SPI_DATA_SIZE_13_BITS                         (12u)              /*!< 13 bit DS */
#define SPI_DATA_SIZE_14_BITS                         (13u)              /*!< 14 bit DS */
#define SPI_DATA_SIZE_15_BITS                         (14u)              /*!< 15 bit DS */
#define SPI_DATA_SIZE_16_BITS                         (15u)              /*!< 16 bit DS */



/**
 * @brief FTLVL[1:0]: FIFO transmission level
 *
 * Note: These bits are set and cleared by hardware.
 * This bit is not used in I2S mode.
 *
 */
#define SPI_TX_FIFO_EMPTY                             (0u)               /*!< FIFO empty */
#define SPI_TX_FIFO_1_4TH                             (1u)               /*!< FIFO 1/4th */
#define SPI_TX_FIFO_1_2ND                             (2u)               /*!< FIFO half */
#define SPI_TX_FIFO_FULL                              (3u)               /*!< FIFO full */

/**
 * @brief SPI FIFO transmission level - bit info
 *
 */
#define SPI_TX_FIFO_LVL_BIT_POS                       (11u)                /*!< FIFO transmission level bit position */
#define SPI_TX_FIFO_LVL_BIT_WIDTH                     (3u)                 /*!< FIFO transmission level width */


/**
 * @brief FRLVL[1:0]: FIFO reception level
 *
 *
 * Note: These bits are set and cleared by hardware.
 * Note: These bits are not used in I2S mode and in
 * SPI receive-only mode while CRC calculation is enabled.
 *
 */
#define SPI_RX_FIFO_EMPTY                             (0u)               /*!< FIFO empty */
#define SPI_RX_FIFO_1_4TH                             (1u)               /*!< FIFO 1/4th */
#define SPI_RX_FIFO_1_2ND                             (2u)               /*!< FIFO half */
#define SPI_RX_FIFO_FULL                              (3u)               /*!< FIFO full */

/**
 * @brief SPI FIFO reception level - bit info
 *
 */
#define SPI_RX_FIFO_LVL_BIT_POS                       (9u)                /*!< FIFO reception level bit position */
#define SPI_RX_FIFO_LVL_BIT_WIDTH                     (3u)                /*!< FIFO reception level width */


/**
 * @brief FRE: Frame format error - bit info
 *
 * This flag is used for SPI in TI slave mode and I2S slave mode.
 * Refer to Section 27.5.11: SPI error flags and
 * Section 27.7.8: I2S error flags. This flag is set by hardware
 * and reset when SPIx_SR is read by software.
 *
 */
#define SPI_FRAME_FORMAT_ERR_BIT_POS                  (8u)                /*!< Frame format error bit position */
#define SPI_FRAME_FORMAT_ERR_BIT_WIDTH                (1u)                /*!< Frame format error width */


/**
 * @brief BSY: Busy flag - bit info
 *
 * This flag is set and cleared by hardware.
 *
 * Note: The BSY flag must be used with caution:
 * refer to Section 27.5.10: SPI status flags
 * and Procedure for disabling the SPI on page 855.
 *
 */
#define SPI_BUSY_FLAG_BIT_POS                         (7u)                /*!< Busy flag bit position */
#define SPI_BUSY_FLAG_BIT_WIDTH                       (1u)                /*!< Busy flag error width */

/**
 * @brief OVR: Over run flag - bit info
 *
 * This flag is set by hardware and reset by a
 * software sequence. Refer to I2S error flags
 * on page 887 for the software sequence.
 *
 */
#define SPI_OVER_RUN_BIT_POS                          (6u)                /*!< Over run flag bit position */
#define SPI_OVER_RUN_BIT_WIDTH                        (1u)                /*!< Over run flag width */


/**
 * @brief MODF: Mode fault - bit info
 *
 * This flag is set by hardware and reset by a
 * software sequence. Refer to Section :
 * Mode fault (MODF) on page 865 for the software sequence.
 *
 * Note: This bit is not used in I2S mode.
 *
 */
#define SPI_MODE_FAULT_BIT_POS                        (5u)                /*!< Mode fault bit position */
#define SPI_MODE_FAULT_BIT_WIDTH                      (1u)                /*!< Mode fault width */


/**
 * @brief CRCERR: CRC error flag - bit info
 *
 *
 * Note: This flag is set by hardware and cleared by software writing 0.
 * This bit is not used in I2S mode.
 *
 */
#define SPI_CRC_ERR_BIT_POS                           (4u)                /*!< CRC error flag bit position */
#define SPI_CRC_ERR_BIT_WIDTH                         (1u)                /*!< CRC error flag width */


/**
 * @brief UDR: Under run flag - bit info
 *
 * This flag is set by hardware and reset by a
 * software sequence. Refer to I2S error flags on
 * page 887 for the software sequence.
 *
 * Note: This bit is not used in SPI mode.
 *
 */
#define SPI_UNDER_RUN_BIT_POS                         (3u)                /*!< Under run flag bit position */
#define SPI_UNDER_RUN_BIT_WIDTH                       (1u)                /*!< Under run flag width */


/**
 * @brief CHSIDE: Channel side
 *
 *
 * Note: This bit is not used in SPI mode.
 * It has no significance in PCM mode.
 *
 */
#define SPI_CHN_TXRX_LEFT_SIDE                        (0u)                /*!< Channel Left has to be transmitted or has been received */
#define SPI_CHN_TXRX_RIGHT_SIDE                       (1u)                /*!< Channel Right has to be transmitted or has been received */

/**
 * @brief Channel side - bit info
 *
 */
#define SPI_CHN_SIDE_BIT_POS                          (2u)                /*!< Channel side bit position */
#define SPI_CHN_SIDE_BIT_WIDTH                        (1u)                /*!< Channel side width */


/**
 * @brief TXE: Transmit buffer empty
 *
 */
#define SPI_TXE_BIT_POS                               (1u)                /*!< Transmit buffer empty bit position */
#define SPI_TXE_BIT_WIDTH                             (1u)                /*!< Transmit buffer empty width */


/**
 * @brief RXNE: Receive buffer not empty
 *
 */
#define SPI_RXNXE_BIT_POS                             (0u)                /*!< Receive buffer not empty bit position */
#define SPI_RXNXE_BIT_WIDTH                           (1u)                /*!< Receive buffer not empty width */


/**
 * @brief SPI Data size bit info
 *
 */
#define SPI_DATA_SIZE_BIT_POS                         (8u)                /*!< SPI Data size bit position */
#define SPI_DATA_SIZE_BIT_WIDTH                       (0xFu)              /*!< SPI Data size bit width */


/**
 * @brief SPI get status flag macro
 *
 */
#define SPI_GET_FLAG_STATUS(spi_reg, bit_mask)        ((spi_reg) & (bit_mask))

/**
 * @brief SPI Initialize
 *
 */
Drv_Status_t LL_HAL_SPI_Init(SPI_Handle_t *hSPI);

/**
 * @brief SPI Peripheral Enable or Disable
 *
 */
Drv_Status_t LL_HAL_SPI_Enable(SPI_Handle_t *hSPI, uint8_t Enable);

/**
 * @brief SPI DeInitialize
 *
 */
Drv_Status_t LL_HAL_SPI_Deinit(SPI_Handle_t *pSPI);

/**
 * @brief SPI Internal slave select
 *
 */
Drv_Status_t LL_HAL_SPI_SSI_Cntrl(SPI_Handle_t *pSPI, uint8_t Enable);

/**
 * @brief SPI Peripheral CLK Control
 *
 */
Drv_Status_t LL_HAL_SPI_PCLK_Cntrl(SPI_Handle_t *pSPI, uint8_t Enable);

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
