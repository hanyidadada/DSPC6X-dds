/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_i2c.c
 *
 * @brief This file contains the lower level function to access I2C device.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

/* CSL Header file */
#include <ti/csl/cslr_i2c.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_chipAux.h>

#include "system/platform.h"
#include "c66x_i2c.h"

#define I2C_OWN_ADDR                 (0x10)
#define I2C_PRESCALER                (16)

/** I2C register base address */
CSL_I2cRegs *i2cRegs = ((CSL_I2cRegs*) CSL_I2C_DATA_CONTROL_REGS);

/**
 * @brief This function calculates I2C clock freq and initializes it
 *
 * @param i2c_iclk I2C input clock
 *
 * @param i2c_freq I2C bus frequency to be set
 *
 * @return NULL
 */
void i2c_set_clockfrq(uint32_t i2c_iclk, uint32_t i2c_master_freq)
{
    uint32_t i2c_module_freq;
    uint32_t i2c_dll_val, i2c_dlh_val;

    /* Set I2C in reset */
    i2cRegs->ICMDR &= (~(1 << CSL_I2C_ICMDR_IRS_SHIFT));

    /* delay 100 cycles */
    cpu_delaycycles(100);

    /* i2c module clock = i2c input clock(KHz) / prescale + 1 */
    i2c_module_freq = (i2c_iclk / 1000) / (I2C_PRESCALER + 1);

    /* i2c master clock divide = (i2c_dll_val + 6) + (i2c_clock_high + 6) */
    /* i2c master clock = i2c module clock / i2c master clock divide */
    i2c_dll_val = ((i2c_module_freq / i2c_master_freq) - 12) / 2;
    i2c_dlh_val = i2c_dll_val;

    /* Set I2C High and Low Clock Hold from above calculation */
    i2cRegs->ICPSC  = I2C_PRESCALER;
    i2cRegs->ICCLKL = i2c_dll_val;
    i2cRegs->ICCLKH = i2c_dlh_val;

    /* Take I2C Out of Reset */
    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_IRS_SHIFT);

    /* delay 100 cycles */
    cpu_delaycycles(100);
}

/**
 * @brief This function initializes I2C controller
 *
 * @param slave_addr address of the slave device
 *
 * @return NULL
 */
void i2c_init(uint32_t slave_addr)
{
    /* Set I2C in reset */
    i2cRegs->ICMDR &= (~(1 << CSL_I2C_ICMDR_IRS_SHIFT));

    /* delay 100 cycles */
    cpu_delaycycles(100);

    /* Set Own Address */
    i2cRegs->ICOAR = I2C_OWN_ADDR;

    /* Enable the Xmt, Master Mode */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_MST_SHIFT) | \
                        (1 << CSL_I2C_ICMDR_TRX_SHIFT) | \
                        (1 << CSL_I2C_ICMDR_FREE_SHIFT));

    /* Set slave device address */
    i2cRegs->ICSAR = slave_addr;

    /* Take I2C Out of Reset */
    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_IRS_SHIFT);

    /* delay 100 cycles */
    cpu_delaycycles(100);
}

/**
 * @brief This function is for I2C send the blocking
 *
 * @param data_cnt counter for data
 *
 * @param slave_data data for sending
 *
 * @return NULL
 */
void i2c_send_blocking(uint32_t data_cnt, uint8_t *slave_data)
{
    uint16_t i;

    /* Set data count vaule */
    i2cRegs->ICCNT = data_cnt;

    /* Enable the Xmt, Master Mode */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_MST_SHIFT) | \
                        (1 << CSL_I2C_ICMDR_TRX_SHIFT) | \
                        (1 << CSL_I2C_ICMDR_STP_SHIFT));

    /* Take I2C Out of Reset */
    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_IRS_SHIFT);

    /* Enable START condition bit */
    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_STT_SHIFT);

    for(i = 0; i < data_cnt; i++) {
        while(!((i2cRegs->ICSTR) & CSL_I2C_ICSTR_ICXRDY_MASK));
        i2cRegs->ICDXR = slave_data[i];
    }

    /* Wait bus free */
    while(((i2cRegs->ICSTR) & CSL_I2C_ICSTR_BB_MASK));
}

/**
 * @brief This function is for I2C receive the blocking
 *
 * @param data_cnt counter for data
 *
 * @param slave_data space for storing received data
 *
 * @return NULL
 */
void i2c_rcv_blocking(uint32_t data_cnt, uint8_t *slave_data)
{
    uint16_t i;

    /* Set data count vaule */
    i2cRegs->ICCNT = data_cnt;

    /* Enable the Xmt, Master Mode */
    i2cRegs->ICMDR = ((1 << CSL_I2C_ICMDR_MST_SHIFT) | \
                        (1 << CSL_I2C_ICMDR_STP_SHIFT));

    /* Take I2C Out of Reset */
    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_IRS_SHIFT);

    /* Enable START condition bit */
    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_STT_SHIFT);

    for(i = 0; i < data_cnt; i++) {
        while(!((i2cRegs->ICSTR) & CSL_I2C_ICSTR_ICRRDY_MASK));
        slave_data[i] = i2cRegs->ICDRR;
    }

    /* Wait bus free */
    while(((i2cRegs->ICSTR) & CSL_I2C_ICSTR_BB_MASK));

    i2cRegs->ICMDR |= (1 << CSL_I2C_ICMDR_STP_SHIFT);
}

/**
 * @brief This function is for I2C write a register
 *
 * @param reg_addr register address
 *
 * @param reg_data register data
 *
 * @return NULL
 */
void i2c_reg_write(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t buff[2] = {0};

    buff[0] = reg_addr;
    buff[1] = reg_data;

    i2c_send_blocking(2, buff);
}

/**
 * @brief This function is for I2C read the register
 *
 * @param reg_addr register address
 *
 * @return NULL
 */
uint8_t i2c_reg_read(uint8_t reg_addr)
{
    uint8_t buff = 0;

    buff = reg_addr;

    i2c_send_blocking(1, &buff);

    /* delay 100000 cycles */
    cpu_delaycycles(100000);

    i2c_rcv_blocking(1, &buff);

    return buff;
}
