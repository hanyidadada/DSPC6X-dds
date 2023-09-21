/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_nor.c
 *
 * @brief This file contains the lower level function to access nor flash.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <ti/csl/csl_types.h>
#include <ti/csl/cslr_device.h>

#include "c66x_spi.h"
#include "c66x_nor.h"



/**
 * @brief This function sends a single byte command and receives response data
 *
 * @param cmd Command sent to the NOR flash
 *
 * @param response Pointer to the RX response data
 *
 * @param len Lenght of the response in bytes
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_cmd(uint8_t cmd, uint8_t *response, uint32_t len)
{
    Bool flags = FALSE;
    int32_t ret;

    if(len == 0) {
        flags = TRUE;
    }

    /* Send the command byte */
    ret = c66x_spi_xfer(1, &cmd, NULL, flags);
    if(ret != 0) {
        printf("SF: Failed to send command %02x: %d\n", cmd, ret);
        return ret;
    }

    /* Receive the response */
    if(len) {
        ret = c66x_spi_xfer(len, NULL, response, TRUE);
        if(ret != 0) {
            printf("SF: Failed to read response (%zu bytes): %d\n", len, ret);
            return ret;
        }
    }

    return ret;
}

/**
 * @brief This function sends a read command and reads data from the flash
 *
 * @param cmd Command sent to the NOR flash
 *
 * @param cmd_len Length of the command in bytes
 *
 * @param dat Pointer to the data read
 *
 * @param data_len Lenght of the data read in bytes
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_cmd_read(uint8_t *cmd, uint32_t cmd_len, \
            uint8_t *data, uint32_t data_len)
{
    Bool flags = FALSE;
    int32_t ret;

    if(data_len == 0) {
        flags = TRUE;
    }

    /* Send read command */
    ret = c66x_spi_xfer(cmd_len, cmd, NULL, flags);
    if(ret != 0) {
        printf("SF: Failed to send read command (%zu bytes): %d\n",cmd_len, ret);
        return ret;
    } else if(data_len != 0) {
        /* Read data */
        ret = c66x_spi_xfer(data_len, NULL, data, TRUE);
        if(ret != 0) {
            printf("SF: Failed to read %zu bytes of data: %d\n",data_len, ret);
            return ret;
        }
    }

    return ret;
}

/**
 * @brief This function sends a write command and writes data to the flash
 *
 * @param cmd Command sent to the NOR flash
 *
 * @param cmd_len Length of the command in bytes
 *
 * @param dat Pointer to the data to be written
 *
 * @param data_len Lenght of the data in bytes
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_cmd_write(uint8_t *cmd, uint32_t cmd_len, \
            uint8_t *data, uint32_t data_len)
{
    Bool flags = FALSE;
    int32_t ret;

    if(data_len == 0) {
        flags = TRUE;
    }

    /* Send write command */
    ret = c66x_spi_xfer(cmd_len, cmd, NULL, flags);
    if(ret != 0) {
        printf("SF: Failed to send write command (%zu bytes): %d\n", cmd_len, ret);
        return ret;
    } else if(data_len != 0) {
        /* Write data */
        ret = c66x_spi_xfer(data_len, data, NULL, TRUE);
        if(ret != 0) {
            printf("SF: Failed to write %zu bytes of data: %d\n", data_len, ret);
            return ret;
        }
    }

    return ret;
}

/**
 * @brief This function check the flash ready status
 *
 * @param timeout timeout count for ready status detect
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
static int32_t c66x_nor_wait_ready(uint32_t  timeout)
{
    int32_t ret;
    uint8_t status;
    uint8_t cmd = SPI_NOR_CMD_RDSR;

    do {
        /* Send Read Status command */
        ret = c66x_spi_xfer(1, &cmd, NULL, FALSE);
        if(ret)
            goto err_out;

        /* Read status value */
        ret = c66x_spi_xfer(1, NULL, &status, TRUE);
        if(ret)
            goto err_out;

        /* Write-in-Progress complete */
        if((status & SPI_NOR_SR_WIP) == 0)
            break;

        if(timeout-- == 0)
            break;

    } while(TRUE);

    if((status & SPI_NOR_SR_WIP) != 0) {
        /* Timed out */
        ret = -1;
        goto err_out;
    }

    return 0;

err_out:
    return ret;
}

/**
 * @brief This function reads data from the NOR flash
 *
 * @param info struct for nor flash hardware information
 *
 * @param addr offset address of the NOR flash
 *
 * @param len lenth in bytes to read
 *
 * @param buf pointer of the buffer to store the bytes read
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_read(nor_device_info *info, uint32_t addr, \
            uint32_t len, uint8_t *buf)
{
    uint8_t cmd[4];
    int32_t ret;

    /* Validate address input */
    if(addr + len > info->max_flash_size) {
        printf("write/read size is larger than max flash size\n");
        ret = -1;
        goto err_out;
    }

    /* Initialize the command to be sent serially */
    cmd[0] = SPI_NOR_CMD_READ;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    ret = c66x_nor_cmd_read(cmd, 4, buf, len);
    if(ret)
        goto err_out;

    return 0;

err_out:
    return ret;
}

/**
 * @brief This function writes data from the NOR flash
 *
 * @param info struct for nor flash hardware information
 *
 * @param addr offset address of the NOR flash
 *
 * @param len lenth in bytes to write
 *
 * @param buf pointer of the buffer to store the bytes write
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_write(nor_device_info *info, uint32_t addr, \
            uint32_t len, uint8_t *buf)
{
    uint32_t page_addr, byte_addr;
    uint32_t page_size;
    uint32_t loopCount;
    uint32_t chunk_len;
    uint32_t actual;
    int32_t ret;
    uint8_t cmd[4];

    /* Validate address input */
    if(addr + len > info->max_flash_size) {
        printf("write/read size is larger than max flash size\n");
        ret = -1;
        goto err_out;
    }

    page_size = info->page_size;
    page_addr = addr / page_size;
    byte_addr = addr & (info->page_size - 1); /* % page_size; */

    for(actual = 0; actual < len; actual += chunk_len) {
        /* Send Write Enable command */
        ret = c66x_nor_cmd(SPI_NOR_CMD_WREN, NULL, 0);
        if(ret)
            goto err_out;

        /* Send Page Program command */
        chunk_len = ((len - actual) < (page_size - byte_addr) ?
            (len - actual) : (page_size - byte_addr));

        cmd[0] = SPI_NOR_CMD_PP;
        cmd[1] = (uint8_t)(addr >> 16);
        cmd[2] = (uint8_t)(addr >> 8);
        cmd[3] = (uint8_t)addr;

        ret = c66x_nor_cmd_write(cmd, 4, buf + actual, chunk_len);
        if(ret)
            goto err_out;

        ret = c66x_nor_wait_ready(SPI_NOR_PROG_TIMEOUT);
        if(ret)
            goto err_out;

        page_addr++;
        addr += chunk_len;
        byte_addr = 0;

        loopCount = 4000;
        while(loopCount--) {
            asm("   NOP");
        }
    }

    return 0;

err_out:
    return ret;
}

/**
 * @brief This function erase sector of NOR flash
 *
 * @param sector_number Sector number to erase
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_erase_sector(nor_device_info *info, uint32_t sector_number)
{
    int32_t ret;
    uint8_t cmd[4];
    uint32_t cmd_len;
    uint32_t address;

    /*
     * This function currently uses sector erase only.
     * probably speed things up by using bulk erase
     * when possible.
     */

    if(sector_number >= info->sector_count) {
        printf("sector number is out of max nor flash sector count\n");
        ret = -1;
        goto err_out;
    } else {
        address = sector_number * info->sector_size;
        cmd[0] = SPI_NOR_CMD_SE;
        cmd[1] = (address >> 16) & 0xff;
        cmd[2] = (address >>  8) & 0xff;
        cmd[3] = (address >>  0) & 0xff;

        cmd_len = 4;
    }

    /* Send Write Enable command */
    ret = c66x_nor_cmd(SPI_NOR_CMD_WREN, NULL, 0);
    if(ret)
        goto err_out;

    ret = c66x_nor_cmd_write(cmd, cmd_len, NULL, 0);
    if(ret)
        goto err_out;

    ret = c66x_nor_wait_ready(SPI_NOR_SECTOR_ERASE_TIMEOUT);
    if(ret)
        goto err_out;

    return 0;

err_out:
    return ret;
}

/**
 * @brief This function erase all sectors of NOR flash
 *
 * @param void
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_erase_bulk(void)
{
    int32_t ret;
    uint8_t cmd[4];
    uint32_t cmd_len;

    cmd[0] = SPI_NOR_CMD_BE;
    cmd_len = 1;

    /* Send Write Enable command */
    ret = c66x_nor_cmd(SPI_NOR_CMD_WREN, NULL, 0);
    if(ret)
        goto err_out;

    ret = c66x_nor_cmd_write(cmd, cmd_len, NULL, 0);
    if(ret)
        goto err_out;

    ret = c66x_nor_wait_ready(SPI_NOR_SECTOR_ERASE_TIMEOUT);
    if(ret)
        goto err_out;

    return 0;

err_out:
    return ret;
}

/**
 * @brief This function get the nor flash hardware information
 *
 * @param info struct fo store nor flash hardware information
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
int32_t c66x_nor_get_info(nor_device_info *info)
{
    int32_t ret;
    uint8_t idcode[3];     /* Initialize the SPI interface */

    /* Read the ID codes */
    ret = c66x_nor_cmd(SPI_NOR_CMD_RDID, idcode, sizeof(idcode));
    if(ret) {
        printf("c66x_nor_get_info: Error in reading the idcode\n");
        goto err_out;
    }

    info->manufacturer_id = idcode[0];
    info->memory_type = idcode[1];
    info->memory_capacity = idcode[2];

    if(info->manufacturer_id == N25QX_MANUFACTURE_ID) {
        /* Expected Manufacturer ID does not match */
        printf("manufacturer id 0x%02x detected !\n", info->manufacturer_id);
    } else if (info->manufacturer_id == GD25X_MANUFACTURE_ID) {
        /* Expected Manufacturer ID does not match */
        printf("manufacturer id 0x%02x detected !\n", info->manufacturer_id);
        info->sector_count = GD25Q128_SPI_NOR_SECTOR_COUNT;
        info->sector_size = GD25Q_SPI_NOR_SECTOR_SIZE;
        info->page_size = GD25Q_SPI_NOR_PAGE_SIZE;
        info->max_flash_size = info->sector_count * info->sector_size;

        return 0;
    } else {
        /* Expected Manufacturer ID does not match */
        printf("error manufacturer id 0x%02x detected !\n", info->manufacturer_id);
        ret = -1;
        goto err_out;
    }

    switch(info->memory_capacity) {
    case 0x16:
        info->sector_count = N25Q32_SPI_NOR_SECTOR_COUNT;
        info->sector_size = N25Q_SPI_NOR_SECTOR_SIZE;
        info->page_size = N25Q_SPI_NOR_PAGE_SIZE;
        info->max_flash_size = info->sector_count * info->sector_size;
        break;
    case 0x17:
        info->sector_count = N25Q64_SPI_NOR_SECTOR_COUNT;
        info->sector_size = N25Q_SPI_NOR_SECTOR_SIZE;
        info->page_size = N25Q_SPI_NOR_PAGE_SIZE;
        info->max_flash_size = info->sector_count * info->sector_size;
        break;
    case 0x18:
        info->sector_count = N25Q128_SPI_NOR_SECTOR_COUNT;
        info->sector_size = N25Q_SPI_NOR_SECTOR_SIZE;
        info->page_size = N25Q_SPI_NOR_PAGE_SIZE;
        info->max_flash_size = info->sector_count * info->sector_size;
        break;
    default:
        /* Expected meomry capacity does not match */
        printf("error memory capacity id 0x%02x detected !\n", info->memory_capacity);
        break;
    }

    return 0;

err_out:
    return ret;
}
