/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_i2c.h
 *
 * @brief This file is the header file for I2C module.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#ifndef _C66X_I2C_H_
#define _C66X_I2C_H_

/* ================ Function declarations ================ */
void i2c_set_clockfrq(uint32_t i2c_iclk, uint32_t i2c_master_freq);
void i2c_init(uint32_t slave_addr);
void i2c_reg_write(uint8_t reg_addr, uint8_t reg_data);
uint8_t i2c_reg_read(uint8_t reg_addr);

#endif /* #ifndef _C66X_I2C_H_ */
