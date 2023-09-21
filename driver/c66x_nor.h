/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_nor.h
 *
 * @brief This file is the header file for NOR module.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#ifndef C66X_NOR_H
#define C66X_NOR_H
#include "c66x_spi.h"
/* SPI NOR Hardware Info */
/* Numonyx Manufacture ID assigned by JEDEC */
#define GD25X_MANUFACTURE_ID              0xC8
#define N25QX_MANUFACTURE_ID              0x20
/* total number of data sectors on the spi flash */
#define N25Q32_SPI_NOR_SECTOR_COUNT       64
#define N25Q64_SPI_NOR_SECTOR_COUNT       128
#define N25Q128_SPI_NOR_SECTOR_COUNT      256
#define GD25Q128_SPI_NOR_SECTOR_COUNT     256
/* Number of bytes in single page, suitable for N25Qx/GD25Qx spi flash */
#define N25Q_SPI_NOR_PAGE_SIZE            256
#define GD25Q_SPI_NOR_PAGE_SIZE           256
/* number of bytes in single sector, suitable for N25Qx/GD25Qx spi flash */
#define N25Q_SPI_NOR_SECTOR_SIZE          65536    /* Number of bytes in a data sector */
#define GD25Q_SPI_NOR_SECTOR_SIZE         65536    /* Number of bytes in a data sector */

/* SPI NOR Commands */
#define SPI_NOR_CMD_RDID           0x9f     /* Read manufacture/device ID */
#define SPI_NOR_CMD_WREN           0x06     /* Write enable */
#define SPI_NOR_CMD_WRDI           0x04     /* Write Disable */
#define SPI_NOR_CMD_RDSR           0x05     /* Read Status Register */
#define SPI_NOR_CMD_WRSR           0x01     /* Write Status Register */
#define SPI_NOR_CMD_READ           0x03     /* Read data */
#define SPI_NOR_CMD_FAST_READ      0x0B     /* Read data bytes at higher speed */
#define SPI_NOR_CMD_PP             0x02     /* Page Program */
#define SPI_NOR_CMD_SSE            0x20     /* Sub Sector Erase */
#define SPI_NOR_CMD_SE             0xd8     /* Sector Erase */
#define SPI_NOR_CMD_BE             0xc7     /* Bulk Erase */

#define SPI_NOR_SR_WIP             (1 << 0)   /* Status Register, Write-in-Progress bit */
#define SPI_NOR_BE_SECTOR_NUM      (uint32_t)-1 /* Sector number set for bulk erase */

/* Read status Write In Progress timeout */
#define SPI_NOR_PROG_TIMEOUT          5000
#define SPI_NOR_PAGE_ERASE_TIMEOUT    500000
#define SPI_NOR_SECTOR_ERASE_TIMEOUT  150000000

typedef struct {
    uint32_t manufacturer_id; /* manufacturer id */
    uint32_t memory_type;   /* memory type */
    uint32_t memory_capacity; /* memory capacity */
    uint32_t page_size; /* number of bytes in a data page */
    uint32_t sector_count; /* total number of sectors */
    uint32_t sector_size; /* number of bytes in a data sector */
    uint32_t max_flash_size; /* total number of bytes in device */
} nor_device_info;

int32_t c66x_nor_read(nor_device_info *info, uint32_t addr, uint32_t len, uint8_t *buf);
int32_t c66x_nor_write(nor_device_info *info, uint32_t addr, uint32_t len, uint8_t *buf);
int32_t c66x_nor_erase_sector(nor_device_info *info, uint32_t sector_number);
int32_t c66x_nor_erase_bulk(void);
int32_t c66x_nor_get_info(nor_device_info *info);

#endif
