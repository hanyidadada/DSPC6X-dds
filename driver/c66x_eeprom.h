/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_eeprom.h
 *
 * @brief This file is the header file for EEPROM.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#ifndef C66X_EEPROM_H
#define C66X_EEPROM_H
#include <stdint.h>
/** EEPROM page writer buffer size */
#define EEPROM_PAGE_SIZE 256
#define TMP102_I2C_SLAVE_ADDR           0x49
#define AT24C1024B_I2C_SLAVE_ADDR       0x51

/* used for EEPROM test */
#define EEPROM_TEST_OFFSET      0
#define EEPROM_TEST_SIZE        0x8000

#define MAX_WAIT_TIME 10000

int16_t c66x_eeprom_page_write(uint16_t byte_addr, uint8_t uchEepromI2cAddress, uint8_t *puiData, \
    uint32_t uiNumBytes, uint32_t uiEndBusState);
int16_t c66x_eeprom_sequential_read ( uint16_t byte_addr, uint32_t uiNumBytes, \
    uint8_t *puiData, uint8_t uchEepromI2cAddress);
void EEpromInit(uint32_t mainPllFreq);
#endif
