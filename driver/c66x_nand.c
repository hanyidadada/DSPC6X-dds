/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_nand.c
 *
 * @brief This file contains the lower level function to access nand flash.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#include <string.h>
#include <stdio.h>

#include <ti/csl/csl_pscAux.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/cslr_emif16.h>
#include <ti/csl/soc.h>

#include "system/platform.h"
#include "c66x_nand.h"



static uint8_t nand_page_buf[BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE];

/**
 * @brief This function is used to indicates command cycle occurring
 * and to send command to NAND device
 *
 * @param cmd Command to NAND
 *
 * @return NULL
 */
static void nand_cmd(uint32_t cmd)
{
    /* 8-bit NAND */
    uint8_t *cle_addr;

    cle_addr = (uint8_t *)NAND_CMD_ADDR;

    *cle_addr = cmd;
}

/**
 * @brief This function is used to indicates Address cycle occurring
 * and to send address value to NAND device
 *
 * @param addr Address offser to NAND
 *
 * @return NULL
 */
static void nand_ale(uint32_t addr)
{
    /* 8-bit NAND */
    uint8_t *ale_addr;

    ale_addr = (uint8_t *)NAND_ALE_ADDR;

    *ale_addr = addr;
}

/**
 * @brief This function waits for the NAND status to be ready
 *
 * @param in_timeout time out value in micro seconds
 *
 * @return
 *     @retval 0 successful
 *     @retval -1 failed
 */
 static int32_t nand_wait_ready(uint32_t in_timeout)
 {
    uint32_t count = 0;

    do {
        cpu_delaycycles(5000);

        if((CSL_FEXT(hEmif16Cfg->NANDFSR, EMIF16_NANDFSR_WAITSTAT) & 1) == 1) {
            break;
        }

        count ++;
    } while(count < in_timeout);

    if(count >= in_timeout)
        return -1;

    return 0;
 }

/**
 * @brief This function is used to read Nand data byte
 *
 * @param puch_value Pointer to data buffer
 *
 * @return NULL
 */
static void nand_read_data_byte(uint8_t* puch_value)
{
    /*8-bit NAND*/
    uint8_t *data_addr;

    data_addr = (uint8_t *)NAND_DATA_ADDR;

    *puch_value = *data_addr;
}

/**
 * @brief This function is used to read data bytes from the NAND device
 *
 * @param num_bytes Number of bytes to be read
 *
 * @param p_buffer Pointer to data buffer
 *
 * @return 0
 */
static uint32_t nand_read_data_bytes(uint32_t num_bytes, uint8_t *p_buffer)
{
    uint32_t i;

    /* 8-bit NAND */
    for(i = 0; i < num_bytes; i++) {
        /* NANDRead done directly without checking for nand width */
        nand_read_data_byte((uint8_t *)p_buffer);
        p_buffer++;
    }

    return 0;
}

/**
 * @brief This function is used to write Nand data byte
 *
 * @param data Pointer to data buffer
 *
 * @return NULL
 */
static void nand_write_data_byte(uint8_t data)
{
    volatile uint8_t *dest;

    /* Data is written to the data register on the rising edge of WE# when
     * CE#, CLE, and ALE are LOW, and
     * the device is not busy.
     */
    dest = (volatile uint8_t *)(NAND_DATA_ADDR);
    *dest = data;
}

/**
 * @brief This function is used to write data bytes from the NAND device
 *
 * @param num_bytes Number of bytes to be write
 *
 * @param p_buffer Pointer to data buffer
 *
 * @return 0
 */
uint32_t nand_write_data_bytes(uint32_t num_bytes, uint8_t *p_buffer)
{
    uint32_t i;

    for(i = 0; i < num_bytes; i++) {
        /* NAND Write done directly without checking for nand width */
        nand_write_data_byte((uint8_t) *p_buffer);
        p_buffer++;
    }

    return 0;
}

/**
 * @brief This function is used to read raw ECC code after writing to NAND
 *
 * @param code Pointer to ecc buffer
 *
 * @return NULL
 */
static void nand_read_4bit_ecc(uint32_t *code)
{
    uint32_t mask = 0x03ff03ff;

    code[0] = hEmif16Cfg->NANDF4BECC1R & mask;
    code[1] = hEmif16Cfg->NANDF4BECC2R & mask;
    code[2] = hEmif16Cfg->NANDF4BECC3R & mask;
    code[3] = hEmif16Cfg->NANDF4BECC4R & mask;
}

/**
 * @brief This function is used to calculate ECC code
 *
 * @param read dummy read flag
 *
 * @param ecc_code Pointer to ecc buffer
 *
 * @return NULL
 */
static void nand_cal_4bit_ecc(uint32_t read, uint8_t *ecc_code)
{
    uint32_t i, raw_ecc[4], *p;

    /* After a read, terminate ECC calculation by a dummy read
     * of some 4-bit ECC register.  ECC covers everything that
     * was read; correct() just uses the hardware state, so
     * ecc_code is not needed.
     */
    if(read) {
        i = hEmif16Cfg->NANDF4BECC1R;
        return;
    }

    /* Pack eight raw 10-bit ecc values into ten bytes, making
     * two passes which each convert four values (in upper and
     * lower halves of two 32-bit words) into five bytes.  The
     * ROM boot loader uses this same packing scheme.
     */
    nand_read_4bit_ecc(raw_ecc);
    for(i = 0, p = raw_ecc; i < 2; i++, p += 2) {
        *ecc_code++ =   p[0]        & 0xff;
        *ecc_code++ = ((p[0] >>  8) & 0x03) | ((p[0] >> 14) & 0xfc);
        *ecc_code++ = ((p[0] >> 22) & 0x0f) | ((p[1] <<  4) & 0xf0);
        *ecc_code++ = ((p[1] >>  4) & 0x3f) | ((p[1] >> 10) & 0xc0);
        *ecc_code++ =  (p[1] >> 18) & 0xff;
    }

    return;
}

/**
 * @brief This function is used to Correct up to 4 bits
 * Correct up to 4 bits in data we just read, using state left in the
 * hardware plus the ecc_code computed when it was first written.
 *
 * @param data Pointer to error value
 *
 * @param ecc_code Pointer to ecc buffer
 *
 * @return
 *     @retval corrected number of correct data
 *     @retval -1 failed
 */
static int32_t nand_verify_4bit_ecc(uint8_t *data, uint8_t *ecc_code)
{
    int32_t i;
    uint16_t ecc10[8];
    uint16_t *ecc16;
    uint32_t syndrome[4];
    uint32_t num_errors, corrected;

    /* All bytes 0xff?  It's an erased page; ignore its ECC. */
    for(i = 0; i < 10; i++) {
        if (ecc_code[i] != 0xff)
            goto compare;
    }
    return 0;

compare:
    /*
     * Unpack ten bytes into eight 10 bit values.  We know we're
     * little-endian, and use type punning for less shifting/masking.
     */
    ecc16 = (uint16_t *)ecc_code;

    ecc10[0] =  (ecc16[0] >>  0) & 0x3ff;
    ecc10[1] = ((ecc16[0] >> 10) & 0x3f) | ((ecc16[1] << 6) & 0x3c0);
    ecc10[2] =  (ecc16[1] >>  4) & 0x3ff;
    ecc10[3] = ((ecc16[1] >> 14) & 0x3)  | ((ecc16[2] << 2) & 0x3fc);
    ecc10[4] =  (ecc16[2] >>  8)         | ((ecc16[3] << 8) & 0x300);
    ecc10[5] =  (ecc16[3] >>  2) & 0x3ff;
    ecc10[6] = ((ecc16[3] >> 12) & 0xf)  | ((ecc16[4] << 4) & 0x3f0);
    ecc10[7] =  (ecc16[4] >>  6) & 0x3ff;

    /* Tell ECC controller about the expected ECC codes. */
    for(i = 7; i >= 0; i--) {
        hEmif16Cfg->NANDF4BECCLR = ecc10[i];
    }

    /* Allow time for syndrome calculation ... then read it.
     * A syndrome of all zeroes 0 means no detected errors.
     */
    i = hEmif16Cfg->NANDFSR;
    nand_read_4bit_ecc(syndrome);
    if(!(syndrome[0] | syndrome[1] | syndrome[2] | syndrome[3]))
        return 0;

    /*
     * Clear any previous address calculation by doing a dummy read of an
     * error address register.
     */
    i = hEmif16Cfg->NANDFEA1R;

    /*
     * Start address calculation, and wait for it to complete.
     * We _could_ start reading more data while this is working,
     * to speed up the overall page read.
     */
    CSL_FINS(hEmif16Cfg->NANDFCTL, EMIF16_NANDFCTL_ADDR_CALC_ST, 1);
    for(;;) {
        uint32_t fsr = hEmif16Cfg->NANDFSR;

        switch((fsr >> 8) & 0x0f) {
        case 0:     /* no error, should not happen */
            i = hEmif16Cfg->NANDFEV1R;
            return 0;
        case 1:     /* five or more errors detected */
            i = hEmif16Cfg->NANDFEV1R;
            return -1;
        case 2:     /* error addresses computed */
        case 3:
            num_errors = 1 + ((fsr >> 16) & 0x03);
            goto correct;
        default:    /* still working on it */
            cpu_delaycycles(25000);
            continue;
        }
    }

correct:
    /* correct each error */
    for(i = 0, corrected = 0; i < num_errors; i++) {
        int error_address, error_value;

        if(i > 1) {
            error_address = hEmif16Cfg->NANDFEA2R;
            error_value = hEmif16Cfg->NANDFEV2R;;
        } else {
            error_address = hEmif16Cfg->NANDFEA1R;
            error_value = hEmif16Cfg->NANDFEV1R;
        }

        if(i & 1) {
            error_address >>= 16;
            error_value >>= 16;
        }
        error_address &= 0x3ff;
        error_address = (512 + 7) - error_address;

        if(error_address < 512) {
            data[error_address] ^= error_value;
            corrected++;
        }
    }

    return corrected;
}

/**
 * @brief This function is used to read Nand spare area
 *
 * @param block_addr Block Address
 *
 * @param page Page Number
 *
 * @param p_buffer Pointer to Data Buffer
 *
 * @return
 *     @retval 0 successfully
 *     @retval -1 failed
 */
int32_t nand_read_spare(uint32_t block_addr, uint32_t page, uint8_t *p_buffer)
{
    uint32_t uiAddr, ret_val = 0;
    uint32_t uiColumn;
    uint8_t status;


    /* Read the data to the destination buffer and detect error */
    uiColumn = BYTES_PER_PAGE;
    /* Send 0x50h command to read the spare area */
    nand_cmd(NAND_PAGE_READ); // First cycle send 0
    cpu_delaycycles(50000);

    /*
     * Send address of the block + page to be read
     * Address cycles = 4, Block shift = 22, Page Shift = 16, Bigblock = 0
     */
    uiAddr = PACK_ADDR(uiColumn, page, block_addr);
    nand_ale((uiAddr >>  0u) & 0xFF);   // CA0-CA7 1st Cycle; column addr
    nand_ale((uiAddr >>  8u) & 0x0F);   // CA8-CA11 and Upper Nibble Low 2nd Cycle; colum  addr
    nand_ale((uiAddr >> 16u) & 0xFF);   // PA0-PA5 and BA9-BA8 3rd Cycle;Page addr & Block addr
    nand_ale((uiAddr >> 24u) & 0xFF);   // BA0-BA7 4th Cycle; Plane addr

    nand_cmd(0x30); // Last cycle send 30h command
    cpu_delaycycles(50000);

    // Wait for Ready Busy Pin to go HIGH
    ret_val = nand_wait_ready(EMIF16_NAND_PROG_TIMEOUT);
    if (ret_val != 0)
        return -1;

    /* Read the data to the destination buffer and detect error */
    nand_read_data_bytes(64, p_buffer);

    nand_cmd(NAND_STATUS);
    cpu_delaycycles(50000);
    nand_read_data_byte(&status);

    /* if SR0 bit is set to 1, there is Error - operation failed */
    if ((status & 0x01) == 1)
        return -1;

    return 0;
}

/**
 * @brief This function is used to write Nand spare area
 *
 * @param block_addr Block Address
 *
 * @param page Page Number
 *
 * @param p_buffer Pointer to Data Buffer
 *
 * @return
 *     @retval 0 successfully
 *     @retval -1 failed
 */
int32_t nand_write_spare(uint32_t block_addr, uint32_t page, uint8_t *p_buffer)
{
    uint32_t addr;
    uint32_t ret_val = 0;
    uint8_t  status;
    uint8_t *pBuffer_loc;
    uint32_t uiColumn;

    /* Read the data to the destination buffer and detect error */
    uiColumn = BYTES_PER_PAGE;

    /* Spare Area*/
    nand_cmd(NAND_PROG_PAGE);
    cpu_delaycycles(50000);

    /*
     * Send address of the block + page to be read
     * Address cycles = 4, Block shift = 22, Page Shift = 16, Bigblock = 0
     */
    addr = PACK_ADDR(uiColumn, page, block_addr);

    nand_ale((addr >>  0u) & 0xFF);   // CA0-CA7  1st Cycle;  column addr
    nand_ale((addr >>  8u) & 0x0F);   // CA8-CA11and Upper Nibble Low 2nd Cycle;  colum  addr
    nand_ale((addr >> 16u) & 0xFF);   // PA0-PA5 and BA9 -BA8 3rd Cycle;Page addr & Block addr
    nand_ale((addr >> 24u) & 0xFF);    // BA-7 to BA0  4th Cycle;  Plane addr

    cpu_delaycycles(50000);

    /* Write the data */
    pBuffer_loc = p_buffer;
    nand_write_data_bytes(SPARE_BYTES_PER_PAGE, (uint8_t *)pBuffer_loc);

    /* Wait for Ready Busy Pin to go HIGH  */
    nand_cmd(NAND_CMD_10H);

    cpu_delaycycles(50000);

    ret_val = nand_wait_ready(EMIF16_NAND_PROG_TIMEOUT*50);

    if(ret_val != 0)
        return -1;

    nand_cmd(NAND_STATUS);
    cpu_delaycycles(50000);

    nand_read_data_byte(&status);

    /* if SR0 bit is set to 1, there is Error - operation failed */
    if((status & 0x01) == 1)
        return -1;

    return 0;
}

/**
 * @brief This function is used to erases the specified block of NAND flash
 *
 * @param block_addr Block Address
 *
 * @return
 *     @retval 0 successfully
 *     @retval -1 failed
 */
int32_t nand_erase_block(uint32_t block_num)
{
    uint32_t addr = 0, ret_val = 0;
    uint8_t  status;

    nand_cmd(NAND_BLOCK_ERASE); // Block erase command
    cpu_delaycycles(125000);

    /*
     * Send address of the block + page to be read
     * Address cycles = 2, Block shift = 22, Page shiht = 16
     */
    addr = PACK_ADDR(0x0, 0x0, block_num);

    /* Properly adjust the shifts to match to the data sheet */
    nand_ale((addr >> 16u) & 0xC0);   // A17-A24 3rd Cycle; Block addr
    cpu_delaycycles(125000);
    nand_ale((addr >> 24u) & 0xFF);   // A25-A26 4th Cycle; Plane addr
    cpu_delaycycles(5000000);

    nand_cmd(NAND_ERASE_CONFIRM); // Erase confirm
    cpu_delaycycles(50000);

    /* Wait for erase operation to finish: 2msec  */
    ret_val = nand_wait_ready(EMIF16_NAND_BLOCK_ERASE_TIMEOUT);

    if(ret_val != 0)
        return -1;

    nand_cmd(NAND_STATUS);
    cpu_delaycycles(50000);

    nand_read_data_byte(&status);

    /* if SR0 bit is set to 1, there is Error - operation failed */
    if((status & 0x01) == 1)
        return -1;

    return 0;
}

/**
 * @brief This function is used to reads a page from NAND flash and detects and
 * corrects the bit errors if ECC is enabled
 *
 * @param address Block Address/Page address of NAND flash
 *
 * @param p_buffer Pointer to buffer
 *
 * @return
 *     @retval 0 successfully
 *     @retval -1 failed
 */
 int32_t nand_read_page(nand_addr address, uint8_t* p_buffer)
 {
    int32_t i = 0;
    uint8_t *puchSpareAreaBuf;
    uint8_t *pBuffer_loc;
    uint32_t uiAddr;
    uint32_t ret_val = 0;
    int32_t iIteration;
    int32_t byte_count = 0;
    /* ECC locations for the micron nand device */
    uint8_t eccLoc[4 * NAND_MAX_NUM_ECC_BYTES] = {7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
                                                17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
                                                27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
                                                37, 38, 39, 40, 41, 42, 43, 44, 45, 46};
    uint8_t eccCalc[4 * NAND_MAX_NUM_ECC_BYTES];
    uint8_t eccCode[4 * NAND_MAX_NUM_ECC_BYTES];
    if(p_buffer == NULL)
        return -1;

    pBuffer_loc = nand_page_buf;
    puchSpareAreaBuf = nand_page_buf + BYTES_PER_PAGE;
    if (nand_read_spare(address.block_addr, address.page_addr, \
        puchSpareAreaBuf) != 0)
        return -1;
    cpu_delaycycles(50000);

    /*
     * Send address of the block + page to be read
     * Address cycles = 4, Block shift = 10, Page Shift = 6
     */
    for(iIteration = 0; iIteration < 4; iIteration++) {

        uiAddr = PACK_ADDR(byte_count, address.page_addr, address.block_addr);

        /***********************READ A Command***************************************/
        nand_cmd(NAND_PAGE_READ); // First cycle send 0
        cpu_delaycycles(50000);

        nand_ale((uiAddr >>  0u) & 0xFF);   // CA0-CA7  1st Cycle;  column addr
        nand_ale((uiAddr >>  8u) & 0x0F);   // CA8-CA11 and Upper Nibble Low 2nd Cycle;  colum  addr
        nand_ale((uiAddr >> 16u) & 0xFF);   // PA0-PA5 and BA9 -BA8 3rd Cycle;Page addr & Block addr
        nand_ale((uiAddr >> 24u) & 0xFF);    // BA7 to BA0  4th Cycle;  Block addr

        nand_cmd(0x30); // Last cycle send 30h command
        cpu_delaycycles(50000);

        /* Wait for Ready Busy Pin to go HIGH  */
        ret_val = nand_wait_ready(EMIF16_NAND_PROG_TIMEOUT);

        if(ret_val != 0)
            return -1;

        /* Start 4-bit ECC HW calculation for read */
        CSL_FINS(hEmif16Cfg->NANDFCTL, EMIF16_NANDFCTL_4BIT_ECC_ST , 1);

        /* Read the data to the destination buffer and detect error */
        nand_read_data_bytes(512, pBuffer_loc);
        cpu_delaycycles(50000);

        /* Calculate the 4-bit ECC bytes for read */
        nand_cal_4bit_ecc(TRUE, &eccCalc[iIteration*10]);

        CSL_FINS(hEmif16Cfg->NANDFCTL, EMIF16_NANDFCTL_4BIT_ECC_ST , 0);

        for(i = 0; i < NAND_MAX_NUM_ECC_BYTES; i++) {
            eccCode[(iIteration * 10) + i] = puchSpareAreaBuf[eccLoc[(iIteration * 10) + i]];
        }

        if(nand_verify_4bit_ecc(pBuffer_loc, &eccCode[iIteration * 10]) < 0)
            return -1;
        pBuffer_loc += 512;
        byte_count += 512;

    }
    memcpy(p_buffer, nand_page_buf, BYTES_PER_PAGE);

   return 0;
}

/**
 * @brief This function is used to writes a page of NAND flash and detects and
 * corrects the bit errors if ECC is enabled
 *
 * @param address Block Address/Page address of NAND flash
 *
 * @param p_buffer Pointer to buffer
 *
 * @return
 *     @retval 0 successfully
 *     @retval -1 failed
 */
int32_t nand_write_page(nand_addr address, uint8_t* p_buffer)
{
    int32_t iErrors = 0;
    int32_t i = 0;
    int32_t iIteration;
    uint8_t puchSpareAreaBuf[SPARE_BYTES_PER_PAGE];
    uint8_t *pBuffer_loc;
    uint32_t addr;
    uint32_t ret_val = 0;
    uint8_t status;
    int32_t byte_count = 0;

    /* ECC locations for the Numonyx nand device */
    uint8_t eccLoc[4*NAND_MAX_NUM_ECC_BYTES] = { 7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
                                                17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
                                                27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
                                                37, 38, 39, 40, 41, 42, 43, 44, 45, 46
                                                };
    uint8_t eccCalc[4*NAND_MAX_NUM_ECC_BYTES];

    /* Init the buffer by reading the existing values in the spare area */
    iErrors = nand_read_spare(address.block_addr, address.page_addr, puchSpareAreaBuf);
    if(iErrors != 0)
        return iErrors;

    for(iIteration = 0; iIteration < 4; iIteration++) {
        nand_cmd(NAND_PROG_PAGE);
        cpu_delaycycles(50000);

        /*
         * Send address of the block + page to be read
         * Address cycles = 4, Block shift = 22, Page Shift = 16, Bigblock = 0
         */
        addr = PACK_ADDR(byte_count, address.page_addr, address.block_addr);

        nand_ale((addr >>  0u) & 0xFF);   // CA0-CA7  1st Cycle;  column addr
        nand_ale((addr >>  8u) & 0x0F);   // CA8-CA11and Upper Nibble Low 2nd Cycle;  colum  addr
        nand_ale((addr >> 16u) & 0xFF);   // PA0-PA5 and BA9 -BA8 3rd Cycle;Page addr & Block addr
        nand_ale((addr >> 24u) & 0xFF);   // BA-7 to BA0  4th Cycle;  Plane addr

        cpu_delaycycles(50000);

        /* Start 4-bit ECC HW calculation for write */
        CSL_FINS(hEmif16Cfg->NANDFCTL, EMIF16_NANDFCTL_4BIT_ECC_ST, 1);
        cpu_delaycycles(50000);

        /* Write the data */
        pBuffer_loc = p_buffer;
        nand_write_data_bytes(512, pBuffer_loc);

        cpu_delaycycles(50000);

        /* Calculate the 4-bit ECC bytes for write */
        nand_cal_4bit_ecc(FALSE, &eccCalc[iIteration * 10]);

        /* Update the calculated ECC bytes to spare area data */
        for(i = 0; i < NAND_MAX_NUM_ECC_BYTES; i++) {
            puchSpareAreaBuf[eccLoc[(iIteration*10) + i]] = eccCalc[(iIteration * 10) + i];
        }

        byte_count += 512;
        p_buffer += 512;

        /* Wait for Ready Busy Pin to go HIGH  */
        nand_cmd(NAND_CMD_10H);

        cpu_delaycycles(50000);

        ret_val = nand_wait_ready(EMIF16_NAND_PROG_TIMEOUT*50);

        if(ret_val != 0)
            return -1;

        nand_cmd(NAND_STATUS);
        cpu_delaycycles(50000);

        nand_read_data_byte(&status);

        /* if SR0 bit is set to 1, there is Error - operation failed */
        if((status & 0x01) == 1)
            return -1;
    }

    if (nand_write_spare (address.block_addr, address.page_addr, \
        puchSpareAreaBuf) != 0)
        return -1;

    return 0;
}

/**
 * @brief This function is used to get details of the NAND flash used from
 * the id and the table of NAND
 *
 * @param nand_info Pointer to Nand Info structure
 *
 * @return 0
 */
uint32_t nand_get_info(nand_device_info *nand_info)
{
    /* Clear the Information */
    nand_info->device_id = nand_info->manufacturer_id = 0x0;

    /* Read manufacturer ID and device ID */
    nand_cmd(NAND_RDID);
    cpu_delaycycles(50000);
    nand_ale(NAND_ADD_00H);
    cpu_delaycycles(50000);

    /* Always reading the ID alone in 8 bit mode */
    nand_read_data_byte((uint8_t *)&nand_info->manufacturer_id);
    nand_read_data_byte((uint8_t *)&nand_info->device_id);

    nand_info->block_count = BLOCKS_PER_DEVICE;
    nand_info->page_count = PAGES_PER_BLOCK;
    nand_info->page_size = BYTES_PER_PAGE;

    return 0;
}

/**
 * @brief This function is used to initialize nand flash
 *
 * @param void
 *
 * @return
 *     @retval 0 successfully
 *     @retval -1 failed
 */
int8_t nand_init()
{
    uint8_t  status;

    /* Config nand FCR reg. 8 bit NAND, 4-bit HW ECC */
    hEmif16Cfg->A0CR = (0 \
                        | (0 << 31)     /* selectStrobe */ \
                        | (0 << 30)     /* extWait (never with NAND) */ \
                        | (0xf << 26)   /* writeSetup  10 ns */ \
                        | (0x3f << 20)  /* writeStrobe 40 ns */ \
                        | (7 << 17)     /* writeHold   10 ns */ \
                        | (0xf << 13)   /* readSetup   10 ns */ \
                        | (0x3f << 7)   /* readStrobe  60 ns */ \
                        | (7 << 4)      /* readHold    10 ns */ \
                        | (3 << 2)      /* turnAround  40 ns */ \
                        | (0 << 0));    /* asyncSize   8-bit bus */

    CSL_FINS(hEmif16Cfg->NANDFCTL, EMIF16_NANDFCTL_CE0NAND, \
                CSL_EMIF16_NANDFCTL_CE0NAND_ENABLE);
    CSL_FINS(hEmif16Cfg->NANDFCTL, EMIF16_NANDFCTL_4BIT_ECC_SEL, \
                CSL_EMIF16_NANDFCTL_4BIT_ECC_SEL_RESETVAL);

    /* Set the wait polarity */
    hEmif16Cfg->AWCCR = (0x80   /* max extended wait cycle */ \
                        | (0 << 16)     /* CS2 uses WAIT0 */ \
                        | (0 << 28));   /* WAIT0 polarity low */

    /*
     * Wait Rise.
     * Set to 1 by hardware to indicate rising edge on the
     * corresponding WAIT pin has been detected.
     * The WP0-3 bits in the Async Wait Cycle Config register have
     * no effect on these bits.
     */

    /*
     * Asynchronous Timeout.
     * Set to 1 by hardware to indicate that during an extended
     * asynchronous memory access cycle, the WAIT signal did not
     * go inactive within the number of cycles defined by the
     * MAX_EXT_WAIT field in Async Wait Cycle Config register.
     */

    /* clear async timeout *//* clear wait rise */
    hEmif16Cfg->IRR = (1 | (1 << 2));

    nand_cmd(NAND_RST);
    cpu_delaycycles(50000);

    if(nand_wait_ready(EMIF16_NAND_RESET_TIMEOUT) != 0) {
        return -1;
    }

    nand_cmd(NAND_STATUS);
    cpu_delaycycles(50000);

    nand_read_data_byte(&status);
    if((status & 0x01) == 1) {
        /* if SR0 bit is set to 1, there is Error - operation failed */
        return -1;
    }

    return 0;
}
