/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_eeprom.c
 *
 * @brief This file contains the lower level function to access EEPROM.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#include <ti/csl/cslr_i2c.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_chipAux.h>

#include "system/platform.h"

#include "c66x_i2c.h"
#include "c66x_eeprom.h"

/* ================ Defines and Macros ================ */
#define BOOTBITMASK(x,y)            ((( (1 << ((x)-(y)+1) ) - 1 )) << (y))
#define BOOT_READ_BITFIELD(z,x,y)   ((z) & BOOTBITMASK(x,y)) >> (y)
#define I2C_REG_STR_FIELD_BB(x)     BOOT_READ_BITFIELD((x), 12, 12)
#define I2C_REG_STR_FIELD_NACK(x)   BOOT_READ_BITFIELD((x),  1,  1)
#define I2C_REG_STR_FIELD_ARDY(x)   BOOT_READ_BITFIELD((x),  2,  2)
#define I2C_REG_STR_FIELD_XRDY(x)   BOOT_READ_BITFIELD((x),  4,  4)
#define I2C_REG_STR_FIELD_RRDY(x)   BOOT_READ_BITFIELD((x),  3,  3)

/* Timeout limit for master receiver */
#define I2C_MAX_MASTER_RECEIVE_TIMEOUT   24

/* Timeout limit for master transmitter */
#define I2C_MAX_MASTER_TRANSMITTER_TIMEOUT  24

/*
 * Timeout limit after a master transmitter operation is
 * complete, and waiting for access to the MMRs. This should be on
 * the order of two bytes, for the last two that are being sent
 * (one in the shift register, one in the dxr. The units are in bits.
 */
#define I2C_MAX_MASTER_TRANSMITTER_ARDY_TIMEOUT  32

static CSL_I2cRegs *i2cRegs = ((CSL_I2cRegs*) CSL_I2C_DATA_CONTROL_REGS);

/**
 * @brief This function write data to eeprom
 *
 * @param offset offset address of EEPROM
 *
 * @param uchEepromI2cAddress I2C bus slave address
 *
 * @param puiData pointer of the buffer to store the bytes write
 *
 * @param uiNumBytes lenth in bytes to write
 *
 * @param uiEndBusState release bus flag, 0: not release 1: realse
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 no ACK
 *     @retval -2 timeout
 */
int16_t c66x_eeprom_page_write(uint16_t offset, uint8_t uchEepromI2cAddress, \
    uint8_t *puiData, uint32_t uiNumBytes, uint32_t uiEndBusState)
{
    uint32_t uiTimeoutCounter, uiPollingStatus;
    uint32_t uiValue, uiStatusReg, uiCount;
    uint32_t i;

    /* Check for the bus busy signal */
    uiTimeoutCounter = 0;
    do {
        /* Get bus status */
        uiStatusReg = i2cRegs->ICSTR;
        uiPollingStatus = I2C_REG_STR_FIELD_BB(uiStatusReg);

        if (uiPollingStatus) {
            /* Delay 5 us */
            cpu_delaycycles (5000);

            uiTimeoutCounter += 1;
            if (uiTimeoutCounter >= MAX_WAIT_TIME) {
                /* Return to slave receiver, clear nack and bus busy */
                i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                    (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
                return -2;
            }
        } else {
            /* The bus is free */
            uiTimeoutCounter = 0;
        }
    } while (uiTimeoutCounter != 0);

    /* Enter master transmitter mode, set the slave address register */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                        (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                        (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                        (1 << CSL_I2C_ICMDR_TRX_SHIFT)|
                        (1 << CSL_I2C_ICMDR_MST_SHIFT));
    i2cRegs->ICSAR = (uint8_t)uchEepromI2cAddress;
    /* Delay 50 us */
    cpu_delaycycles(5000);

    /* Put the first byte into the transmit register, set the start bit */
    uiValue = (offset >> 8) & 0x00ff;
    i2cRegs->ICDXR =  uiValue;

    /* Set the start bit */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                        (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                        (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                        (1 << CSL_I2C_ICMDR_TRX_SHIFT)|
                        (1 << CSL_I2C_ICMDR_MST_SHIFT)|
                        (1 << CSL_I2C_ICMDR_STT_SHIFT));

    /* Clear timeout counter */
    uiTimeoutCounter = 0;
    for(i = 0; i < I2C_MAX_MASTER_TRANSMITTER_TIMEOUT; i ++) {
        uiStatusReg = i2cRegs->ICSTR;

        if (I2C_REG_STR_FIELD_NACK(uiStatusReg)) {
            /* Return to slave receiver, clear nack and bus busy */
            i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                (1 << CSL_I2C_ICMDR_TRX_SHIFT));
            i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
            return -1;
        }

        if (I2C_REG_STR_FIELD_XRDY(uiStatusReg)) {
            uiValue = offset & 0x00ff;
            i2cRegs->ICDXR =  uiValue;
            break;
        } else {
            /* Delay 5 us */
            cpu_delaycycles (5000);
            uiTimeoutCounter ++;
        }
    }

    if(uiTimeoutCounter == I2C_MAX_MASTER_TRANSMITTER_TIMEOUT) {
        /* Return to slave receiver, clear nack and bus busy */
        i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                            (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                            (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                            (1 << CSL_I2C_ICMDR_TRX_SHIFT));
        i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                            (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
        return -2;
    }

    for(uiCount = 0; uiCount < uiNumBytes; uiCount++) {
        uiTimeoutCounter = 0;
        do {
            /* Read status */
            uiStatusReg = i2cRegs->ICSTR;

            /* On Nack return failure */
            if (I2C_REG_STR_FIELD_NACK(uiStatusReg)) {
                /* Return to slave receiver, clear nack and bus busy */
                i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                    (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
                return -1;
            } else if (I2C_REG_STR_FIELD_XRDY(uiStatusReg)) {
                uiTimeoutCounter = 0;
                uiValue = (*puiData) & 0x00ff;
                puiData += 1;
                /*Write data Transmit Data Register */
                i2cRegs->ICDXR = uiValue;
            } else {
                /* XRDY bit not set */
                /* Delay 5 us */
                cpu_delaycycles (5000);
                uiTimeoutCounter += 1;

                if (uiTimeoutCounter >= I2C_MAX_MASTER_TRANSMITTER_TIMEOUT) {
                    /* Return to slave receiver, clear nack and bus busy */
                    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                        (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                        (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                        (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                    i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                        (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
                    return -2;
                }
            }
        }while (uiTimeoutCounter != 0);
    }

    /* If releasing the bus, send a stop bit */
    if (!uiEndBusState) {
        /* Wait for the ardy bit to go high */
        uiTimeoutCounter = 0;
        do {
            uiStatusReg = i2cRegs->ICSTR;
            if (I2C_REG_STR_FIELD_ARDY(uiStatusReg)) {
                i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_MST_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_STP_SHIFT));
                i2cRegs->ICSTR = (1 << CSL_I2C_ICSTR_BB_SHIFT);
                /* Delay 5 us */
                cpu_delaycycles (5000);
                uiTimeoutCounter = 0;
            } else {
                /* Registers not ready for access */
                uiTimeoutCounter += 1;
                if (uiTimeoutCounter >= I2C_MAX_MASTER_TRANSMITTER_ARDY_TIMEOUT) {
                    /* On timeout put the peripheral into reset, wait, then
                    * take it out of reset */
                    i2cRegs->ICMDR = (1 << CSL_I2C_ICMDR_FREE_SHIFT);
                    /* Delay 5 us */
                    cpu_delaycycles (5000);
                    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                        (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                        (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                        (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                    return -2;
                }
                /* Delay 5 us */
                cpu_delaycycles (5000);
            }
        }while (uiTimeoutCounter != 0);
    } /* end bus release */

    return 0;
}

/**
 * @brief This function read data from eeprom
 *
 * @param offset offset address of EEPROM
 *
 * @param uchEepromI2cAddress I2C bus slave address
 *
 * @param puiData pointer of the buffer to store the bytes read
 *
 * @param uiNumBytes lenth in bytes to read
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 no ACK
 *     @retval -2 timeout
 */
int16_t c66x_eeprom_sequential_read (uint16_t offset, uint32_t uiNumBytes, \
    uint8_t *puiData, uint8_t uchEepromI2cAddress)
{
    uint32_t  uiStatusReg, uiTimeoutCounter, iCount;
    uint16_t uiReturnValue, ushValue;

    /* Write the byte address to the eeprom. Do not send a stop */
    uiReturnValue = c66x_eeprom_page_write (offset, uchEepromI2cAddress, NULL, \
        0, 1);
    if (uiReturnValue)
        return uiReturnValue;

    /* Delay 10 us */
    cpu_delaycycles (10000);

    /* Set the start bit, begin the master read */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                        (1 << CSL_I2C_ICMDR_STT_SHIFT)|
                        (1 << CSL_I2C_ICMDR_MST_SHIFT)|
                        (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                        (1 << CSL_I2C_ICMDR_IRS_SHIFT));

    for (iCount = 0; iCount < uiNumBytes; iCount++) {
        uiTimeoutCounter = 0;
        do {
            /* Read status */
            uiStatusReg = i2cRegs->ICSTR;

            /* On Nack return failure */
            if (I2C_REG_STR_FIELD_NACK(uiStatusReg)) {
                /* Return to slave receiver, clear nack and bus busy */
                i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                    (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
                return -1;
            }

            /* Check for receive byte ready */
            if (I2C_REG_STR_FIELD_RRDY(uiStatusReg)) {
                ushValue = i2cRegs->ICDRR & 0x00ff;
                uiTimeoutCounter = 0;
                *puiData = ushValue;
                puiData++;
            } else {   /* RRDY bit not set */
                /* Delay 5 us */
                cpu_delaycycles (5000);
                uiTimeoutCounter += 1;

                if (uiTimeoutCounter >= I2C_MAX_MASTER_RECEIVE_TIMEOUT)  {
                /* Return to slave receiver, clear nack and bus busy */
                i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                    (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
                return -2;
                }
            }
        } while (uiTimeoutCounter != 0);
    } /* end for loop */

    /* The data block has been read. Send the stop bit */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                        (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                        (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                        (1 << CSL_I2C_ICMDR_MST_SHIFT)|
                        (1 << CSL_I2C_ICMDR_STP_SHIFT));

    /* Wait for the rrdy and read the dummy byte */
    uiTimeoutCounter = 0;
    do {
        uiStatusReg = i2cRegs->ICSTR;

        /* Check for receive byte ready */
        if (I2C_REG_STR_FIELD_RRDY(uiStatusReg)) {
            ushValue = i2cRegs->ICDRR & 0x00ff;
            uiTimeoutCounter = 0;
        } else {  /* rrdy not set */
            /* Delay 5 us */
            cpu_delaycycles (5000);
            uiTimeoutCounter += 1;

            if (uiTimeoutCounter >= I2C_MAX_MASTER_RECEIVE_TIMEOUT) {
                /* Return to slave receiver, clear nack and bus busy */
                i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_FREE_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_RM_SHIFT) |
                                    (1 << CSL_I2C_ICMDR_IRS_SHIFT)|
                                    (1 << CSL_I2C_ICMDR_TRX_SHIFT));
                i2cRegs->ICSTR = ((1 << CSL_I2C_ICSTR_BB_SHIFT) |
                                    (1 << CSL_I2C_ICSTR_NACKSNT_SHIFT));
                return -2;
            }
        }
    }while (uiTimeoutCounter != 0);

    return 0;
}

void EEpromInit(uint32_t mainPllFreq)
{
	i2c_set_clockfrq(mainPllFreq/6, 400);
    //cpu_delaycycles(1000000);
	return;
}
