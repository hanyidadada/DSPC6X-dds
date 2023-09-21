/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_spi.h
 *
 * @brief This file is the header file for spi module.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#ifndef C66X_SPI_H
#define C66X_SPI_H
#include <stdint.h>
#include <stdbool.h>
#include <ti/csl/csl_types.h>
int32_t c66x_spi_init(uint32_t cs, uint32_t spi_iclk, uint32_t freq);
void c66x_spi_disable(void);
uint32_t c66x_spi_xfer(uint32_t bitlen, uint8_t *dout, uint8_t *din, Bool flags);
int32_t c66x_spi_read_word(Uint16 *cmd_buf, uint32_t cmd_len, Uint16 *data_buf, uint32_t data_len);
int32_t c66x_spi_write_word(Uint16 *cmd_buf, uint32_t cmd_len, Uint16 *data_buf, uint32_t data_len);

#endif
