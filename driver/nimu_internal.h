/*
 * nimu_internal.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *   @file  nimu_internal.h
 *
 *   @brief
 *      Network Interface Module Ethernet driver adaptation layer. These are internal
 *      definitions for the driver.
 *
 */
/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file nimu_internal.h
 *
 * @brief Network Interface Module Ethernet driver adaptation layer.
 * These are internal definitions for the driver.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

#ifndef _NIMU_INTERNAL_H_
#define _NIMU_INTERNAL_H_

#include <ti/csl/tistdtypes.h>
#include <ti/ndk/inc/stkmain.h>

/* CSL EMAC include */
#include <ti/csl/csl_cpsw.h>
#include <ti/csl/csl_cpsgmii.h>
#include <ti/csl/csl_cpsgmiiAux.h>
#include <ti/csl/csl_mdio.h>
#include <ti/csl/csl_mdioAux.h>

/* BootCfg module include */
#include <ti/csl/csl_bootcfg.h>
#include <ti/csl/csl_bootcfgAux.h>

/* CSL CHIP, SEM Functional layer includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>

/* CSL Cache module includes */
#include <ti/csl/csl_cacheAux.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* NetCP includes */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/pa/pa.h>


/* Standard C-native includes  */
#include <stdlib.h>
#include <string.h>

/* XDC/BIOS includes */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* Number of cores on c6698 */
#define     NUM_CORES           8


/*
 * Pre-Pad Packet Data Offset
 *
 *   The TCP/IP stack library requires that every packet device
 *   include enough L2 header room for all supported headers. In
 *   order to support PPPoE, this requires a 22 byte L2 header.
 *   Thus, since standard Ethernet is only 14 bytes, we must add
 *   on an additional 8 byte offset, or PPPoE can not function
 *   with our driver.
*/
#define     PKT_PREPAD                      8

/* Indicates whether RAM based multicast lists are suported for this
 * EMAC peripheral.
 */
#define     RAM_MCAST                       0

/* Indicates whether HASH based multicasting is suported for this
 * EMAC peripheral.
 */
#define     HASH_MCAST                      0

/* Multicast Address List Size */
#define     PKT_MAX_MCAST                   31

/** Number of ports in the ethernet subsystem */
#define         NUM_PORTS                   3u

/** Number of MAC/GMII ports in the ethernet switch */
#define         NUM_MAC_PORTS               2u


/** Host descriptor size.
 *
 *  Big enough to hold the mandatory fields of the
 *  host descriptor and PA Control Data
 *
 *  = 32 bytes for Host desc + PA Control data + 16 bytes padding
 */
#define         SIZE_HOST_DESC              MAX_DESC_SIZE

/* High Priority QM Rx Interrupt Threshold */
#define     RX_INT_THRESHOLD            4u


/* Accumulator channel to use */
#define     PA_ACC_CHANNEL_NUM                  0u
#define     PA_ACC_CHANNEL_NUM_SEC_INTERFACE    8u

/* Max Number of Rx packets in the Queue before passing to NDK */
#define     MAX_NUM_RAW_PKTS_INQ                    4

#define     NUM_EMAC_INTERFACES         2

/* Rx queue (one for all PKT devices) */
#ifndef _INCLUDE_NIMU_CODE
extern PBMQ    PBMQ_rx;
#endif

/**
 * @brief   External memory start address
 */

#define     EMAC_EXTMEM                          0x80000000
#define     EMAC_LL2SRAM                         0x00800000
#define     EMAC_MSMCSRAM                        0x0c000000

/**
 * @brief
 *  Packet device information
 *
 * @details
 *  This structure caches the device info.
 */
typedef struct _pdinfo
{
    /**
     * @brief       Physical index of this device (0 to n-1).
     */
    uint32_t            PhysIdx;
    /*
     * @brief       Handle to logical driver.
     */
    void *          hEther;
    /*
     * @brief       Semaphore handle used by NDK stack and driver
     *              to communicate any pending Rx events that need
     *              to be serviced by NDK ethernet stack.
     */
    STKEVENT_Handle hEvent;
    /*
     * @brief       MAC Address
     */
    uint8_t           bMacAddr[6];
    /*
     * @brief       Current RX filter
     */
    uint32_t            Filter;
    /*
     * @brief       Current MCast Address Countr
     */
    uint32_t            MCastCnt;
    /*
     * @brief       Multicast list configured by the Application.
     */
    uint8_t           bMCast[6*PKT_MAX_MCAST];
    /*
     * @brief       Transmitter "free" flag
     */
    uint32_t            TxFree;
    /*
     * @brief       Tx queue (one for each PKT device)
     */
    PBMQ            PBMQ_tx;

    /*
     * @brief       Raw Pkt Tx queue (one for each PKT device)
     */
    PBMQ            PBMQ_rawtx;

#ifdef _INCLUDE_NIMU_CODE
    /*
     * @brief       Rx queue (one for each PKT device)
     */
    PBMQ            PBMQ_rx;

    /*
     * @brief       Raw Pkt Rx queue (one for each PKT device)
     */
    PBMQ            PBMQ_rawrx;
#endif
} PDINFO;

/**
 *  @brief  Nimu_CppiDescCfg
 *
 *          Structure to specify the CPPI descriptor configuration
 *          for a NIMU free queue used to hold pre-allocated
 *          buffers.
 */
typedef struct  _Nimu_CppiDescCfg
{
    /** CPPI Memory region to be used for this set of free descriptors. */
    uint32_t                    descMemRegion;

    /** Number of CPPI free descriptors to allocate */
    uint32_t                    numDesc;

    /** Size of CPPI free descriptors to allocate */
    uint32_t                    descSize;

    /** CPPI Descriptor Type.
     *
     *  Valid values are:
     *      Cppi_DescType_HOST,
     *      Cppi_DescType_MONOLITHIC
     */
    Cppi_DescType               descType;
} Nimu_CppiDescCfg;

/**
 * @brief
 *  NIMU LLD configuration information
 *
 * @details
 *  This structure holds the configurations for the PA, QMSS, CPPI LLDs.
 */
typedef struct _nimu_config
{
    /**
     * @brief       To configure internal or external link ram (0 or 1).
     */
    Bool                    ExtLinkRam;

    /* RX Threshold */

    /** Even ID for the Rx ISR */
    uint32_t                eventID;

    /** Vector ID for the Rx ISR */
    uint32_t                vectorID;

} NIMUConfigParams;

/* The EMAC Initialization Function. */
static int EmacInit (STKEVENT_Handle hEvent);
static int EMACInit_Core (STKEVENT_Handle hEvent);

extern      int32_t       Init_Qmss(void);
extern      int32_t       Init_PASS(void);
extern      int32_t       Init_Cpsw(uint32_t mtu, uint8_t * srcmacaddress);
extern      int32_t       Init_Cppi(void);
extern      int32_t       Verify_Init (void);
extern      void        Init_MDIO(void);
extern      void        Init_SGMII (uint32_t macPortNum);
extern      void        Init_Switch (uint32_t mtu);
extern      int32_t       Switch_update_addr (uint32_t portNum, uint8_t macAddress[6], uint16_t add);
extern      void        Init_MAC(uint32_t, uint8_t *, uint32_t);
extern      int32_t       Setup_Tx(NETIF_DEVICE*     ptr_net_device);
extern      int32_t       Setup_Rx(NETIF_DEVICE*     ptr_net_device);
extern      int32_t       Setup_PASS(void);
extern      int32_t       Add_MACAddress(paEthInfo_t *ethInfo, paRouteInfo_t *routeInfo);
extern      uint32_t      Convert_CoreLocal2GlobalAddr (uint32_t  addr);
extern      void        CycleDelay (int32_t count);
extern      void        EmacRxPktISR (NETIF_DEVICE*     ptr_net_device );
extern      int32_t       Cpsw_SwitchOpen (void);
extern      int32_t       Mdio_Open (void);
extern      int32_t       Sgmii_Open (void);
extern      int32_t       Download_PAFirmware (void);


/**
 * @brief
 *   EMAC_DATA
 *
 * @details
 *  The structure is used to store the private data for the
 *  EMAC controller.
 */
typedef struct EMAC_DATA
{
/**
  * @brief   Private Information
  */
    PDINFO      pdi;
}EMAC_DATA;

static void
EmacRxPkt
(
    uint32_t              port_num,
    Cppi_Desc*          pCppiDesc
);


NIMUConfigParams nimuGlobalConfigParams;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* CPPI/QMSS Handles used by the application */
#pragma DATA_SECTION(gPaTxQHnd,       ".nimu_eth_ll2");
#pragma DATA_SECTION(gTxReturnQHnd, ".nimu_eth_ll2");
#pragma DATA_SECTION(gTxFreeQHnd, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxFreeQHnd, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxQHnd, ".nimu_eth_ll2");
Qmss_QueueHnd gPaTxQHnd [NUM_PA_TX_QUEUES], gTxReturnQHnd[NUM_EMAC_INTERFACES], gTxFreeQHnd[NUM_EMAC_INTERFACES], gRxFreeQHnd[NUM_EMAC_INTERFACES], gRxQHnd[NUM_EMAC_INTERFACES], gTxCmdReturnQHnd[NUM_EMAC_INTERFACES], gTxCmdFreeQHnd[NUM_EMAC_INTERFACES];


/* Queues used */
/* PA command response queue handle */
#pragma DATA_SECTION(gPaCfgCmdRespQHnd, ".nimu_eth_ll2");
Qmss_QueueHnd                           gPaCfgCmdRespQHnd;


/* CPPI Handles */
#pragma DATA_SECTION(gRxFlowHnd, ".nimu_eth_ll2");
Cppi_FlowHnd                            gRxFlowHnd[NUM_EMAC_INTERFACES];

#pragma DATA_SECTION(gPaL2Handles, ".nimu_eth_ll2");
#pragma DATA_SECTION(gPaL3Handles, ".nimu_eth_ll2");
#pragma DATA_SECTION(gPaL4Handles, ".nimu_eth_ll2");
paHandleL2L3_t                          gPaL2Handles[MAX_NUM_L2_HANDLES];
paHandleL2L3_t                          gPaL3Handles[MAX_NUM_L3_HANDLES];
paHandleL4_t                            gPaL4Handles[MAX_NUM_L4_HANDLES];

#pragma DATA_SECTION(gHiPriAccumList, ".nimu_eth_ll2");
#pragma DATA_ALIGN (gHiPriAccumList, 128)
#pragma DATA_SECTION(gHiPriAccumList2, ".nimu_eth_ll2");
#pragma DATA_ALIGN (gHiPriAccumList2, 128)
#define MAX_HI_PRI_ACCUM_LIST_SIZE      32
uint32_t                                  gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE*2];
uint32_t                                  gHiPriAccumList2[MAX_HI_PRI_ACCUM_LIST_SIZE*2];
#pragma DATA_SECTION(gAccChannelNum, ".nimu_eth_ll2");
uint32_t                                  gAccChannelNum;

/* Semaphore Handle associated to receive data */
Semaphore_Handle    ethSemHandle;

/* Various stats  */
#pragma DATA_SECTION(gTxCounter, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxCounter, ".nimu_eth_ll2");
#pragma DATA_SECTION(gTxDropCounter, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxDropCounter, ".nimu_eth_ll2");
uint32_t gTxCounter=0, gRxCounter=0, gTxDropCounter = 0, gRxDropCounter=0;

/* TX port number */
#pragma DATA_SECTION(gTxPort, ".nimu_eth_ll2");
uint32_t gTxPort;

#endif /* _NIMU_ETH_H_ */
