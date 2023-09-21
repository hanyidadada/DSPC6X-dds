/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_spi.c
 *
 * @brief This file contains the lower level function to access spi device.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#include <stdio.h>

#include <ti/csl/cslr_spi.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/cslr_device.h>

#include "system/platform.h"
#include "c66x_spi.h"

static uint32_t data1_reg_val;

/** SPI register base address */
#define spiRegs     ((CSL_SpiRegs *)CSL_SPI_REGS)

/**
 * @brief This function claims the SPI bus in the SPI controller
 *
 * @param cs Chip Select number for the slave SPI device
 *
 * @param spi_iclk SPI input clock
 *
 * @param freq SPI clock frequency to be set
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_spi_init(uint32_t cs, uint32_t spi_iclk, uint32_t freq)
{
    uint32_t scalar;

    /* Enable the SPI hardware */
    spiRegs->SPIGCR0 = CSL_SPI_SPIGCR0_RESET_IN_RESET;
    cpu_delaycycles(2000);
    spiRegs->SPIGCR0 = CSL_SPI_SPIGCR0_RESET_OUT_OF_RESET;

    /* Set master mode, powered up and not activated */
    spiRegs->SPIGCR1 = (CSL_SPI_SPIGCR1_MASTER_MASTER << CSL_SPI_SPIGCR1_MASTER_SHIFT) |
                    (CSL_SPI_SPIGCR1_CLKMOD_INTERNAL << CSL_SPI_SPIGCR1_CLKMOD_SHIFT);


    /* CS0, CS1, CLK, Slave in and Slave out are functional pins */
    if(cs == 0) {
        spiRegs->SPIPC0 = (CSL_SPI_SPIPC0_SCS0FUN0_SPI << CSL_SPI_SPIPC0_SCS0FUN0_SHIFT) |
                        (CSL_SPI_SPIPC0_CLKFUN_SPI << CSL_SPI_SPIPC0_CLKFUN_SHIFT)   |
                        (CSL_SPI_SPIPC0_SIMOFUN_SPI << CSL_SPI_SPIPC0_SIMOFUN_SHIFT) |
                        (CSL_SPI_SPIPC0_SOMIFUN_SPI << CSL_SPI_SPIPC0_SOMIFUN_SHIFT);
    } else if(cs == 1) {
        spiRegs->SPIPC0 =  ((CSL_SPI_SPIPC0_SCS0FUN1_SPI << CSL_SPI_SPIPC0_SCS0FUN1_SHIFT) |
                        (CSL_SPI_SPIPC0_CLKFUN_SPI << CSL_SPI_SPIPC0_CLKFUN_SHIFT)    |
                        (CSL_SPI_SPIPC0_SIMOFUN_SPI << CSL_SPI_SPIPC0_SIMOFUN_SHIFT)  |
                        (CSL_SPI_SPIPC0_SOMIFUN_SPI << CSL_SPI_SPIPC0_SOMIFUN_SHIFT)) & 0xFFFF;
    }

    /* setup format */
    scalar = ((spi_iclk / freq) - 1 ) & 0xFF;

    if(cs == 0) {
        spiRegs->SPIFMT[0] = (8 << CSL_SPI_SPIFMT_CHARLEN_SHIFT) |
                        (scalar << CSL_SPI_SPIFMT_PRESCALE_SHIFT)                      |
                        (CSL_SPI_SPIFMT_PHASE_DELAY << CSL_SPI_SPIFMT_PHASE_SHIFT)     |
                        (CSL_SPI_SPIFMT_POLARITY_LOW << CSL_SPI_SPIFMT_POLARITY_SHIFT) |
                        (CSL_SPI_SPIFMT_SHIFTDIR_MSB << CSL_SPI_SPIFMT_SHIFTDIR_SHIFT);
    } else if(cs == 1) {
        spiRegs->SPIFMT[0] = (8 << CSL_SPI_SPIFMT_CHARLEN_SHIFT) |
                        (scalar << CSL_SPI_SPIFMT_PRESCALE_SHIFT)                      |
                        (CSL_SPI_SPIFMT_PHASE_DELAY << CSL_SPI_SPIFMT_PHASE_SHIFT)  |
                        (CSL_SPI_SPIFMT_POLARITY_LOW << CSL_SPI_SPIFMT_POLARITY_SHIFT) |
                        (CSL_SPI_SPIFMT_SHIFTDIR_MSB << CSL_SPI_SPIFMT_SHIFTDIR_SHIFT);
    }

    /* hold cs active at end of transfer until explicitly de-asserted */
    data1_reg_val = (CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT) |
                    (0x02 << CSL_SPI_SPIDAT1_CSNR_SHIFT);
    if(cs == 0) {
         spiRegs->SPIDAT1 = (CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT) |
                            (0x02 << CSL_SPI_SPIDAT1_CSNR_SHIFT);
    }

    /*
     * including a minor delay. No science here. Should be good even with
     * no delay
     */
    if(cs == 0) {
        spiRegs->SPIDELAY = (8 << CSL_SPI_SPIDELAY_C2TDELAY_SHIFT) |
                        (8 << CSL_SPI_SPIDELAY_T2CDELAY_SHIFT);
        /* default chip select register */
        spiRegs->SPIDEF = CSL_SPI_SPIDEF_RESETVAL;
    } else if(cs == 1) {
        spiRegs->SPIDELAY = (6 << CSL_SPI_SPIDELAY_C2TDELAY_SHIFT) |
                        (3 << CSL_SPI_SPIDELAY_T2CDELAY_SHIFT);
    }

    /* no interrupts */
    spiRegs->SPIINT0 = CSL_SPI_SPIINT0_RESETVAL;
    spiRegs->SPILVL  = CSL_SPI_SPILVL_RESETVAL;

    /* enable SPI */
    spiRegs->SPIGCR1 |= (CSL_SPI_SPIGCR1_ENABLE_ENABLE << \
        CSL_SPI_SPIGCR1_ENABLE_SHIFT);

    if(cs == 1) {
        spiRegs->SPIDAT0 = 1 << 15;
        cpu_delaycycles (10000);
        /* Read SPIFLG, wait untill the RX full interrupt */
        if((spiRegs->SPIFLG & (CSL_SPI_SPIFLG_RXINTFLG_FULL << \
            CSL_SPI_SPIFLG_RXINTFLG_SHIFT))) {
            /* Read one byte data */
            scalar = spiRegs->SPIBUF & 0xFF;
            /* Clear the Data */
            spiRegs->SPIBUF = 0;
        } else {
            /* Read one byte data */
            scalar = spiRegs->SPIBUF & 0xFF;
            return -1;
        }
    }

    return 0;
}

/**
 * @brief This function releases the bus in SPI controller
 *
 * @param void
 *
 * @return NULL
 */
void c66x_spi_disable(void)
{
    /* Disable the SPI hardware */
    spiRegs->SPIGCR1 = CSL_SPI_SPIGCR1_RESETVAL;
}

/**
 * @brief This function sends and receives 8-bit data serially
 *
 * @param nbytes number of bytes of the TX data
 *
 * @param data_out Pointer to the TX data
 *
 * @param data_in Pointer to the RX data
 *
 * @param terminate release/hold cs flag when terminate the transfer
 *
 * @return
 *     @retval 0 successful
 */
uint32_t c66x_spi_xfer(uint32_t nbytes, uint8_t *data_out, \
            uint8_t *data_in, Bool terminate)
{
    uint32_t i, buf_reg;
    uint8_t *tx_ptr;
    uint8_t *rx_ptr;

    tx_ptr = (uint8_t *)data_out;
    rx_ptr = (uint8_t *)data_in;

    /* Clear out any pending read data */
    spiRegs->SPIBUF;

    for(i = 0; i < nbytes; i++) {
        /* Wait untill TX buffer is not full */
        while(spiRegs->SPIBUF & CSL_SPI_SPIBUF_TXFULL_MASK);

        /* Set the TX data to SPIDAT1 */
        data1_reg_val &= ~0xFFFF;
        if(tx_ptr) {
            data1_reg_val |= *tx_ptr & 0xFF;
            tx_ptr++;
        }

        /* Write to SPIDAT1 */
        if((i == (nbytes -1)) && (terminate)) {
            /* Release the CS at the end of the transfer when terminate flag is TRUE */
            spiRegs->SPIDAT1 = data1_reg_val & ~(CSL_SPI_SPIDAT1_CSHOLD_ENABLE << \
                CSL_SPI_SPIDAT1_CSHOLD_SHIFT);
        } else {
            spiRegs->SPIDAT1 = data1_reg_val;
        }

        /* Read SPIBUF, wait untill the RX buffer is not empty */
        while(spiRegs->SPIBUF & (CSL_SPI_SPIBUF_RXEMPTY_MASK));

        /* Read one byte data */
        buf_reg = spiRegs->SPIBUF;
        if(rx_ptr) {
            *rx_ptr = buf_reg & 0xFF;
            rx_ptr++;
        }
    }

    return 0;
}

/**
 * @brief This function sends a read command and reads data in 16-bit data format
 *
 * @param cmd_buf Pointer to the command sent
 *
 * @param cmd_len Length of the command in words
 *
 * @param data_buf Pointer to the data read
 *
 * @param data_len Lenght of the data read in words
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_spi_read_word(uint16_t *cmd_buf, uint32_t cmd_len, \
            uint16_t *data_buf, uint32_t data_len)
{
    uint32_t data1_reg;
    uint16_t *tx_ptr;
    uint16_t *rx_ptr;

    tx_ptr = (uint16_t *)cmd_buf;
    rx_ptr = (uint16_t *)data_buf;

    /*
     * disable the SPI communication by setting
     * the SPIGCR1.ENABLE to 0
     *
     * SPIGCR1
     * ============================================
     * Bit  Field   Value       Description
     * 24   ENABLE  0           SPI disable
     * ============================================
     */
    data1_reg = 0x1 << 24;
    data1_reg = ~data1_reg;
    spiRegs->SPIGCR1 &= data1_reg;

    /*
     * clean TX data into SPIDAT0
     *
     * SPIDAT0
     * ============================================
     * Bit  Field   Value       Description
     * 15-0 TXDATA  0-FFFFh     SPI transmit data
     * ============================================
     *
     */
    spiRegs->SPIDAT0 = 0;

    /*
     * Enable the SPI communication by setting
     * the SPIGCR1.ENABLE to 1
     *
     * SPIGCR1
     * ============================================
     * Bit  Field   Value       Description
     * 24   ENABLE  1           SPI enable
     * ============================================
     */
    data1_reg = 0x1 << 24;
    spiRegs->SPIGCR1 = (spiRegs->SPIGCR1 | data1_reg);

    {
        {
            spiRegs->SPIDAT0 = *tx_ptr;
            cpu_delaycycles(10000);
        }

        /* Read SPIFLG, wait untill the RX full interrupt */
        if((spiRegs->SPIFLG & (CSL_SPI_SPIFLG_RXINTFLG_FULL << \
            CSL_SPI_SPIFLG_RXINTFLG_SHIFT))) {
            /* Read one byte data */
            *rx_ptr = spiRegs->SPIBUF & 0xFF;
            spiRegs->SPIBUF = 0;
        } else {
            return -1;
        }
    }

    return 0;
}

/**
 * @brief This function sends a write command and data in 16-bit data format
 *
 * @param cmd_buf Pointer to the command sent
 *
 * @param cmd_len Length of the command in bytes
 *
 * @param data_buf Pointer to the data read
 *
 * @param data_len Lenght of the data in bytes
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_spi_write_word(uint16_t *cmd_buf, uint32_t cmd_len, \
            uint16_t *data_buf, uint32_t data_len)
{
    uint32_t data1_reg;
    uint16_t *tx_ptr;

    tx_ptr = (uint16_t *)cmd_buf;

    /*
     * disable the SPI communication by setting
     * the SPIGCR1.ENABLE to 0
     *
     * SPIGCR1
     * ============================================
     * Bit  Field   Value       Description
     * 24   ENABLE  0           SPI disable
     * ============================================
     */
    data1_reg = 0x1 << 24;
    data1_reg = ~data1_reg;
    spiRegs->SPIGCR1 &= data1_reg;

    /*
     * clean TX data into SPIDAT0
     *
     * SPIDAT0
     * ============================================
     * Bit  Field   Value       Description
     * 15-0 TXDATA  0-FFFFh     SPI transmit data
     * ============================================
     *
     */
    spiRegs->SPIDAT0 = 0;

    /*
     * Enable the SPI communication by setting
     * the SPIGCR1.ENABLE to 1
     *
     *
     * SPIGCR1
     * ============================================
     * Bit  Field   Value       Description
     * 24   ENABLE  1           SPI enable
     * ============================================
     */
    data1_reg = 0x1 << 24;
    spiRegs->SPIGCR1 = (spiRegs->SPIGCR1 | data1_reg);

    {
        {
            spiRegs->SPIDAT0 = *tx_ptr;
            cpu_delaycycles(10000);
        }

        /* Read SPIFLG, wait untill the RX full interrupt */
        if((spiRegs->SPIFLG & (CSL_SPI_SPIFLG_TXINTFLG_EMPTY << \
            CSL_SPI_SPIFLG_TXINTFLG_SHIFT))) {
            /* Clear the SPIBUF */
            spiRegs->SPIBUF = 0;
            return 0;
        }
        else {
            return -1;
        }
    }
}
