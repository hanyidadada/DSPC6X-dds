/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file c66x_nand.h
 *
 * @brief This file is the header file for NAND module.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#ifndef C66X_NAND_H
#define C66X_NAND_H
#include <stdint.h>
/* NAND FLASH ADDRESS */
#define NAND_DATA_ADDR  ((volatile uint8_t*)0x70000000)  /*emif16_ce0_base */
#define NAND_ALE_ADDR   ((volatile uint8_t*)0x70002000)
#define NAND_CMD_ADDR   ((volatile uint8_t*)0x70004000)
#define NAND_TYPE_MASK_0X00000020   (0x00000020)

/* Macros for delay in micro Sec */
#define STD_DELAY                          (25)
#define EMIF16_NAND_PROG_TIMEOUT           (100000)
#define EMIF16_NAND_RESET_TIMEOUT          (100000)
#define EMIF16_NAND_BLOCK_ERASE_TIMEOUT    (2000000)

/* NAND Flash hardware info */
#define BYTES_PER_PAGE                  (2048)
#define SPARE_BYTES_PER_PAGE            (64)
#define PAGES_PER_BLOCK                 (64)
#define TOTAL_BYTES_PER_PAGE            (BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE)
#define TOTAL_BYTES_PER_BLOCK           (TOTAL_BYTES_PER_PAGE * PAGES_PER_BLOCK)
#define BLOCKS_PER_DEVICE               (1024)

#define BLOCK_SIZE (PAGES_PER_BLOCK * BYTES_PER_PAGE)

/* ECC related macros */
#define ECC_BLOCK_SIZE                  (256)   /* in Bytes */
#define ECC_SPARE_OFFSET                (SPARE_BYTES_PER_PAGE-3*(BYTES_PER_PAGE/ECC_BLOCK_SIZE))

/* NAND FLASH COMMANDS */
#define NAND_ADD_00H                    (0x00)
#define NAND_ADD_08H                    (0x08)
#define NAND_CMD_05H                    (0x05)  /* Random Data Read Command */
#define NAND_CMD_10H                    (0x10)  /* Program Confirm Command */
#define NAND_CMD_30H                    (0x30)
#define NAND_CMD_E0H                    (0xE0)
#define NAND_BLOCK_ERASE                (0x60)  /* Block Erase Command */
#define NAND_ERASE_CONFIRM              (0xD0)  /* Erase Confirm Command */
#define NAND_GET_FEATURES               (0xEE)
#define NAND_OTP_DATA_PROG              (0xA0)
#define NAND_OTP_DATA_PROT              (0xA5)
#define NAND_OTP_DATA_READ              (0xAF)
#define NAND_PAGE_READ                  (0x00)  /* Page Read Command */
#define NAND_PAGE_READ_LAST             (0x3F)  /* Page Read Cache Mode Start Last*/
#define NAND_PAGE_READ_RANDOM           (0x00)
#define NAND_PAGE_READ_SEQUENTIAL       (0x31)  /* page Read Cache mode start */
#define NAND_INT_DATA_MOVE_PROG         (0x85)  /* Program for Internal Data Move */
#define NAND_PROG_PAGE                  (0x80)  /* Program Page Command */
#define NAND_PROG_PAGE_CACHE            (0x80)  /* Program Page command */
#define NAND_RANDOM_DATA_IN             (0x85)  /* Program for internal Data Move */
#define NAND_RANDOM_DATA_READ           (0x00)
#define NAND_INT_DATA_MOVE_READ         (0xA5)
#define NAND_RDID                       (0x90)  /* Read NAND ID Command */
#define NAND_READ_PARAM_PAGE            (0xEC)
#define NAND_STATUS                     (0x70)  /* Read Status command */
#define NAND_READ_UNIQUE_ID             (0xED)
#define NAND_RST                        (0xFF)  /* Reset Command */
#define NAND_RDY                        (0x40)
#define NAND_RDIDADD                    (0x20)

/* Maximum number of ECC bytes per page */
#define NAND_MAX_NUM_ECC_BYTES          10

#define PACK_ADDR(col, page, block) \
    ((col & 0x00000fff) | ((page & 0x0000003f) << 16) | ((block & 0x000003ff) << 22))

/** Emif16 register base address */
#define hEmif16Cfg  ((CSL_Emif16Regs*)CSL_EMIF16_REGS)

typedef struct {
    uint32_t column_addr;
    uint32_t page_addr;
    uint32_t block_addr;
} nand_addr;

typedef struct {
    uint32_t device_id;     /* device id */
    uint32_t manufacturer_id;   /* manufacturer id */
    uint32_t block_count;   /* number of blocks per device */
    uint32_t page_size;     /* number of bytes per device */
    uint32_t page_count;    /* number of pages per block */
} nand_device_info;

int32_t nand_read_page(nand_addr address, uint8_t* puchBuffer);
int32_t nand_write_page(nand_addr address, uint8_t* puchBuffer);
uint32_t nand_get_info(nand_device_info *nand_info);
int32_t nand_erase_block(uint32_t uiBlockNumber);
int8_t nand_init();

#endif
