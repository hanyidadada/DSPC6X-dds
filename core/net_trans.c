/*
 * net_trans.c
 *
 *  Created on: 2023��9��21��
 *      Author: hanyi
 */


/* C standard Header files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS6 include */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

/* NDK include */
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/_stack.h>
#include <ti/ndk/inc/tools/console.h>
#include <ti/ndk/inc/tools/servers.h>

/* csl include */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_bootcfgAux.h>

#ifdef SOC_C6678
#include <ti/csl/csl_cpsgmii.h>
#include <ti/csl/csl_cpsgmiiAux.h>
#endif

#include <pthread.h>
#include "system/platform.h"
#include "system/resource_mgr.h"
#include "core/net_trans.h"

extern int udptransportwork(void *arg);
char *VerStr = "\nTCP/UDP Example Client\n";

char *HostName             = "tidsp";
char *LocalIPAddr          = "0.0.0.0";            // Local device Ip addr
char *REMOTE_IPADDR_STRING = "10.2.25.21";            // Remote device IP addr
char *LocalIPMask          = "255.255.255.0";           // Not used when using DHCP
char *GatewayIP            = "0.0.0.0";                 // Not used when using DHCP
char *DomainName           = "demo.net";                // Not used when using DHCP
char *DNSServer            = "0.0.0.0";                 // Used when set to anything but zero

/* Service Status Reports */
/* Here's a quick example of using service status updates */
static char *TaskName[]  = {"Telnet","HTTP","NAT","DHCPS","DHCPC","DNS"};
static char *ReportStr[] = {"","Running","Updated","Complete","Fault"};
static char *StatusStr[] = {"Disabled","Waiting","IPTerm","Failed","Enabled"};
TransWork_t Net_Works[] = {
    {(void *(*)(void*)) udptransportwork, "MQTTWork", NULL}
};

static void ServiceReport(uint32_t Item, uint32_t Status, uint32_t Report, void * h);

uint8_t DHCP_OPTIONS[] = { DHCPOPT_SERVER_IDENTIFIER, DHCPOPT_ROUTER };

/**
 * @brief enable psc module
 *
 * @param void
 *
 * @return NULL
 */
void psc_init()
{
    /* Set psc as Always on state */
    CSL_PSC_enablePowerDomain(CSL_PSC_PD_ALWAYSON);

    /* Start state change */
    CSL_PSC_startStateTransition(CSL_PSC_PD_ALWAYSON);

    /* Wait until the status change is completed */
    while(!CSL_PSC_isStateTransitionDone(CSL_PSC_PD_ALWAYSON));

    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any
     * PASS device register access. This not required for the simulator. */

    /* Set PASS Power domain to ON */
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_PASS);

    /* Enable the clocks for PASS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_PKTPROC, PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_CPGMAC, PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_Crypto, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_PASS);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_PASS));
}

void init_sgmii (uint32_t macPortNum)
{
    int32_t wait_time;
    CSL_SGMII_ADVABILITY    sgmiiCfg;
    CSL_SGMII_STATUS        sgmiiStatus;

    /* Configure the SERDES, set MPY as 10x mode */
    CSL_BootCfgSetSGMIIConfigPLL(0x00000051);

    /* delay 100 cycles */
    cpu_delaycycles(100);

    /*
     * ENRX: 0x1 - Enable Receive Channel
     * RATE: 0x10 - PLL output clock rate by a factor of 2x
     * ALIGN: 0x01 - Enable Comma alignment
     * EQ : 0xC - Set to 1100b when RATE = 0x10
     * ENOC: 0x1 - Enable offset compensation
     */
    CSL_BootCfgSetSGMIIRxConfig (macPortNum, 0x00700621);
    /*
     * ENRX: 0x1 - Enable Transmit channel
     * RATE: 0x10 - PLL output clock rate by a factor of 2x
     * INVPAIR: 0x0 - Normal polarity
     */
    CSL_BootCfgSetSGMIITxConfig (macPortNum, 0x000108A1);

    /* Wait sgmii serdes configure complete time set as 1ms */
    wait_time = 1000;
    while(wait_time) {
        CSL_SGMII_getStatus(macPortNum, &sgmiiStatus);
        if (sgmiiStatus.bIsLocked == 1) {
            break;
        } else {
            /* delay 1000 cycles */
            cpu_delaycycles(1000);
            wait_time --;
        }
    }

    if(wait_time == 0) {
        printf("configure sgmii0 serdes time out!\n");
        return;
    }

    /* Reset the port before configuring it */
    CSL_SGMII_doSoftReset(macPortNum);
    while(CSL_SGMII_getSoftResetStatus(macPortNum) != 0);

    /*
     * Hold the port in soft reset and set up
     * the SGMII control register:
     *      (1) Disable Master Mode
     *      (2) Enable Auto-negotiation
     */
    CSL_SGMII_startRxTxSoftReset(macPortNum);
    CSL_SGMII_disableMasterMode(macPortNum);
    CSL_SGMII_enableAutoNegotiation(macPortNum);
    CSL_SGMII_endRxTxSoftReset(macPortNum);

    /*
     * Setup the Advertised Ability register for this port:
     *      (1) Enable Full duplex mode
     *      (2) Enable Auto Negotiation
     *      (3) Enable the Link
     */
    sgmiiCfg.linkSpeed      =   CSL_SGMII_1000_MBPS;
    sgmiiCfg.duplexMode     =   CSL_SGMII_FULL_DUPLEX;
    sgmiiCfg.bLinkUp        =   1;
    CSL_SGMII_setAdvAbility(macPortNum, &sgmiiCfg);

    do {
        CSL_SGMII_getStatus(macPortNum, &sgmiiStatus);
    } while(sgmiiStatus.bIsLinkUp != 1);

    /* All done with configuration. Return Now. */
    return;
}

int queue_manager_init(void)
{
    QMSS_CFG_T      qmss_cfg;
    CPPI_CFG_T      cppi_cfg;

    /* Initialize the components required to run this application:
     *  (1) QMSS
     *  (2) CPPI
     *  (3) Packet Accelerator
     */
    /* Initialize QMSS */
    if(CSL_chipReadDNUM() == 0) {
        qmss_cfg.master_core = 1;
    } else {
        qmss_cfg.master_core = 0;
    }
    qmss_cfg.max_num_desc = MAX_NUM_DESC;
    qmss_cfg.desc_size = MAX_DESC_SIZE;
    qmss_cfg.mem_region = Qmss_MemRegion_MEMORY_REGION0;

    if(res_mgr_init_qmss (&qmss_cfg) != 0) {
        printf("Failed to initialize the QMSS subsystem \n");
        return -1;
    } else {
        printf("QMSS successfully initialized \n");
    }

    /* Initialize CPPI */
    if(CSL_chipReadDNUM() == 0) {
        cppi_cfg.master_core = 1;
    } else {
        cppi_cfg.master_core = 0;
    }
    cppi_cfg.dma_num = Cppi_CpDma_PASS_CPDMA;
    cppi_cfg.num_tx_queues = NUM_PA_TX_QUEUES;
    cppi_cfg.num_rx_channels = NUM_PA_RX_CHANNELS;
    if(res_mgr_init_cppi (&cppi_cfg) != 0) {
        printf("Failed to initialize CPPI subsystem \n");
        return -1;
    } else {
        printf("CPPI successfully initialized \n");
    }

    if(res_mgr_init_pass() != 0) {
        printf("Failed to initialize the Packet Accelerator \n");
        return -1;
    } else {
        printf("PA successfully initialized \n");
    }

    return 0;
}

#ifdef TCP_CLIENT
static void TCP_perform_receive()
{
    char            *pBuf;
    void *          hBuffer;
    uint8_t         not_first_packet_rx;
    uint32_t        tsMsec, ts1Msec, startMsec, endMsec;
    int32_t         bytes, size;
    uint32_t          ts, ts1, tn, totalBytes;
    SOCKET          stcp = INVALID_SOCKET;
    SOCKET          stcp_child = INVALID_SOCKET;
    struct timeval  to;
    struct sockaddr_in sin1;

    printf("TCP Receive Task started\n");

    /* Allocate the file environment for this task */
    fdOpenSession(TaskSelf());

    /* Create the main TCP listen socket */
    stcp = socket(AF_INET, SOCK_STREAMNC, IPPROTO_TCP);
    if(stcp == INVALID_SOCKET) {
       printf( "Fail socket, %d\n", fdError());
       goto leave;
    }

    /* Set Port = 10001, IP address = IPAddrSend */
    mmZeroInit(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family = AF_INET;
    sin1.sin_addr.s_addr = inet_addr(LocalIPAddr);
    sin1.sin_port   = NDK_htons(TCP_SERVER_PORT);

    if(bind(stcp, (struct sockaddr *)&sin1, sizeof(sin1)) < 0) {
        fdClose( stcp);
        stcp = INVALID_SOCKET;
        printf("Fail to bind socket, %d\n", fdError());
        goto leave;
    }

    /* If the socket is bound and TCP, start listening */
    if(listen(stcp, 1) < 0) {
        fdClose( stcp );
        printf("Fail to listen on socket, %d\n", fdError());
        stcp = INVALID_SOCKET;
        goto leave;
    }

    size = sizeof(sin1);
    mmZeroInit( &sin1, sizeof(struct sockaddr_in) );
    stcp_child = accept(stcp, (struct sockaddr *)&sin1, &size);
    if(stcp_child == INVALID_SOCKET) {
        printf("Failed accept due to error: %d \n", fdError());
        goto leave;
    }

    /* Configure our socket timeout to be 60 seconds */
    to.tv_sec  = 60;
    to.tv_usec = 0;
    setsockopt(stcp_child, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
    setsockopt(stcp_child, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));

    /* Start timer on first packet received */
    not_first_packet_rx = 1;

    /* Accumulate total bytes received */
    totalBytes =0;

    while(totalBytes <  (TEST_DATA_SIZE * MB)) {
        /* There is data available on the active connection */
        bytes = (int)recvnc(stcp_child, (void **)&pBuf, 0, &hBuffer);

        /* If the connection is closed or got an error, break */
        if(bytes <= 0) {
           break;
        } else {
           recvncfree(hBuffer);
        }
        totalBytes += bytes;

        /* Get start time at first packet received */
        if(not_first_packet_rx) {
            llEnter();
            ts = llTimerGetTime(&tsMsec);
            llExit();
            not_first_packet_rx = 0;
        }
    }

    /* Get ending timestamp */
    llEnter();
    ts1 = llTimerGetTime(&ts1Msec);
    llExit();

    /* Compute total time in milliseconds */
    startMsec  = (ts * 1000) + tsMsec;
    endMsec    = (ts1 * 1000) + ts1Msec;
    tn = endMsec - startMsec;

    if(totalBytes > 0) {
        /* Print out the Receive Results. */
        /* Must send back the ----- line as the applet keys on it */
        printf("------------------------------------------------\n");
        printf("Total Data received: %d bytes\n", totalBytes);
        printf("Start Time         : %d msec\n", startMsec);
        printf("End   Time         : %d msec\n", endMsec);
        printf("Total Time expired : %d msec\n", tn);
        printf("Receive Throughput : %u Mb/s \n", (((totalBytes/tn)*1000)*8)/1000000);
        printf("------------------------------------------------\n");
    } else {
         printf("\n\n.....Failed to receive...please re run the test\n\n");
    }


leave:
    if(stcp_child != INVALID_SOCKET)
       fdClose(stcp_child);

    if(stcp != INVALID_SOCKET)
       fdClose(stcp);

    TaskSleep(2000);

    fdCloseSession(TaskSelf());

    TaskDestroy(TaskSelf());
}

#endif

#ifdef UDP_CLIENT
static void UDP_perform_receive(void)
{
    SOCKET          sudp = INVALID_SOCKET;
    char*           pBuf;
    void *          hBuffer;
    int32_t         bytes;
    uint32_t        tsMsec, ts1Msec, startMsec, endMsec;
    uint32_t        ts, ts1, tn;
    uint32_t        totalBytes = 0;
    int32_t         counter = 0;
    float           lost_rate =0;
    struct timeval  timeout;
    struct sockaddr_in sin1;

    printf( "UDP Receive Task started\n");

    /* Allocate the file environment for this task */
    fdOpenSession(TaskSelf());

    /* Create the main UDP listen socket */
    sudp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sudp == INVALID_SOCKET) {
        printf( "Failed to create the receive socket \n");
        goto leave;
    }

    /* Set Port = 10001, leaving IP address = Any */
    mmZeroInit(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family = AF_INET;
//    sin1.sin_len    = sizeof(sin1);
    sin1.sin_port   = NDK_htons(UDP_SERVER_PORT);

    /* Bind the socket */
    if(bind( sudp, (struct sockaddr *) &sin1, sizeof(sin1)) < 0) {
        printf("Failed to bind the receive socket \n");
        goto leave;
    }

    /* Set a time out for the receive in case there is data loss.. time out is in seconds */
    timeout.tv_sec  = 5;    /* wait up to 5 seconds per packet */
    timeout.tv_usec = 0;
    if(setsockopt(sudp, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        printf( "Failed to set the sockopt for receive socket \n");
        goto leave;
    }

    totalBytes = 0;
    while(totalBytes < (TEST_DATA_SIZE* MB)) {
        bytes = (int)recvnc(sudp, (void **)&pBuf, 0, &hBuffer);

        if(bytes < 0) {
            if(fdError() == NDK_EWOULDBLOCK) {
                break; /* we get "would block" on a receive timeout */
            } else {
                /* Receive failed: Close the session & kill the task */
                printf("Receive failed after total bytes %d Error:%d\n",totalBytes, fdError());
                goto leave;
            }
        } else if(bytes == 0) {
            break;
        }

        /* Check if this is the first received packet */
        if(counter == 0) {
            /* YES. Record the start timer. */
            llEnter();
            ts = llTimerGetTime(&tsMsec);
            llExit();
            counter++;
        }

        /* Clean out the buffer */
        recvncfree(hBuffer);

        /* Increment the statistics. */
        totalBytes = totalBytes + bytes;
    }

    /* Control Comes here only if all the data has been received */
    llEnter();
    ts1 = llTimerGetTime(&ts1Msec);
    llExit();

    /* Compute total time in milliseconds */
    startMsec  = (ts * 1000) + tsMsec;
    endMsec    = (ts1 * 1000) + ts1Msec;
    if(bytes < 0) {
        tn = endMsec - startMsec - (timeout.tv_sec * 1000);
    } else {
        tn = endMsec - startMsec;
    }

    lost_rate = 100.0 - ((float)totalBytes / (TEST_DATA_SIZE*MB) * 100);

    if(totalBytes > 0) {
        /* Print out the Receive Results. */
        /* Must send back the ----- line as the applet keys on it */
        printf("------------------------------------------------\n");
        printf("Total Data received: %d bytes\n", totalBytes);
        printf("Total Loss rate    : %f %%\n", lost_rate);
        printf("Start Time         : %d msec\n", startMsec);
        printf("End   Time         : %d msec\n", endMsec);
        printf("Total Time expired : %d msec\n", tn);
        printf("Receive Throughput : %u Mb/s \n", (((totalBytes/tn)*1000)*8)/1000000);
        printf("------------------------------------------------\n");
    } else {
        printf("\n\n.....Failed to receive...please re run the test\n\n");
    }

leave:
    if(sudp != INVALID_SOCKET)
       fdClose(sudp);

    TaskSleep(2000);

    fdCloseSession(TaskSelf());

    TaskDestroy(TaskSelf());

    return;
}

#endif

#ifdef TCP_SERVER
static void TCP_perform_send()
{
    SOCKET      stcp = INVALID_SOCKET;
    struct      sockaddr_in sin1;
    int32_t     count,bytes, res;
    uint32_t    mbps;
    uint32_t    ts, tsMsec, ts1, ts1Msec, startMsec, endMsec;
    uint32_t    tn, totalBytes, frames_count;
    char        *pBuf = NULL;
    Error_Block errorBlock;


    printf("TCP Transmit Task started\n");

    Error_init(&errorBlock);

    /* Allocate the file environment for this task */
    fdOpenSession(TaskSelf());

    /* Create the main TCP listen socket */
    stcp = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(stcp == INVALID_SOCKET) {
        printf( "Failed to create socket, %d\n", fdError());
        goto leave;
    }

    /* Set Port = 10001, IP address = IPAddrSend */
    mmZeroInit(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family = AF_INET;
    sin1.sin_addr.s_addr = inet_addr(REMOTE_IPADDR_STRING);
    sin1.sin_port   = NDK_htons(TCP_CLIENT_PORT);

    for(count = 0; count < 30; count ++) {
        /* Connect socket */
        res = connect(stcp, (struct sockaddr *) &sin1, sizeof(sin1));
        if(res < 0) {
            TaskSleep(5000);
        } else
            break;
        }

        if(res < 0) {
            printf("Failed to connect to socket, %d\n", fdError());
            goto leave;
        }

        printf("connect successfully\n");

        /* Allocate a working buffer */
        if(!(pBuf = (char *)malloc(TCP_CLIENT_BUFSIZE))) {
            printf("Failed buffer allocation\n");
            goto leave;
    }

    frames_count = TEST_DATA_SIZE * MB / TCP_CLIENT_BUFSIZE;

    printf("    Sending %d frames of %d bytes \n", frames_count, TCP_CLIENT_BUFSIZE);

    totalBytes = 0;

    /* Get timestamp */
    llEnter();
    ts = llTimerGetTime(&tsMsec);
    llExit();

    for(count = 0; count < frames_count; count++) {
        if(((bytes = send(stcp, pBuf, (int)TCP_CLIENT_BUFSIZE, 0 )) < 0)) {
            printf( "send failed (%d)\n",fdError());
            goto leave;
        }

        totalBytes += bytes;
    }

    /* Get timestamp again!! */
    llEnter();
    ts1 = llTimerGetTime(&ts1Msec);
    llExit();

    /* Compute total time in milliseconds */
    startMsec  = (ts * 1000) + tsMsec;
    endMsec    = (ts1 * 1000) + ts1Msec;
    tn = endMsec - startMsec;

    if(totalBytes > 0) {
       /* Print out the Transmit Results. */
        printf("------------------------------------------------\n");
        printf("Total Data transmitted: %d bytes\n", totalBytes);
        printf("Start Time            : %d msec\n", startMsec);
        printf("End   Time            : %d msec\n", endMsec);
        printf("Total Time expired    : %d msec\n", tn);
        mbps = (((totalBytes/tn)*1000)*8)/1000000;
        if(mbps > 1024)
            mbps = 1024;            /* we cant be faster than 1 Gig */
        printf("Transmit Throughput : %u Mb/s\n", mbps);
        printf("------------------------------------------------\n");
    } else {
        printf("\n\n.....Failed to receive...please re run the test\n\n");
    }

leave:
     if(pBuf) {
        free(pBuf);
    }

    // We only get here on an error - close the sockets
    if(stcp != INVALID_SOCKET) {
        fdClose( stcp );
    }

    TaskSleep(2000);

    fdCloseSession(TaskSelf());

    TaskDestroy(TaskSelf());
}
#endif

#ifdef UDP_SERVER
static void UDP_perform_send()
{
    SOCKET              sudp = INVALID_SOCKET;
    struct sockaddr_in  sin1;
    uint32_t            tsMsec, ts1Msec, startMsec, endMsec;
    uint32_t            ts, ts1, tn;
    int32_t             count = 0;
    int32_t             bytes;
    uint32_t            totalBytes = 0 , frames_count = 0, mbps = 0;
    char                *pBuf  = NULL;
    char                *pNcBuf = NULL;
    void *              hBuffer;
    struct timeval      timeout;
    Error_Block         errorBlock;

    Error_init(&errorBlock);

    printf("UDP Transmit Task started\n");

    /* Raise priority to transfer data & wait for the link to come up. */
    TaskSetPri(TaskSelf(), 1);

    /* Allocate the file environment for this task */
    fdOpenSession(TaskSelf());

    /* Create the UDP socket */
    sudp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sudp == INVALID_SOCKET) {
        printf("Failed to create socket, %d\n", fdError());
        goto leave;
    }

    /* Set Port = 10001, leaving IP address = Any */
    mmZeroInit(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family = AF_INET;
//    sin1.sin_len    = sizeof(sin1);
    sin1.sin_addr.s_addr    = inet_addr(LocalIPAddr);
    sin1.sin_port   = NDK_htons(UDP_CLIENT_PORT);

    /* Bind the socket */
    if(bind(sudp, (struct sockaddr *) &sin1, sizeof(sin1)) < 0) {
        printf("Failed to bind the socket \n");
        goto leave;
    }

    /* Set a time out for the receivein case there is data loss.. time out is in seconds */
    timeout.tv_sec  = 120;   /* wait up to 60 seconds for receiver ack */
    timeout.tv_usec = 0;
    if(setsockopt(sudp, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        printf("Failed to set the sockopt for receive socket \n");
        goto leave;
    }

    /* Wait for receiver ack. */
    printf("Waiting for receiver to get ready.....\n");
    bytes = (int)recvnc(sudp, (void **)&pNcBuf, 0, &hBuffer);

    /* Clean out the buffer */
    recvncfree(hBuffer);

    // Set Port = 10001, IP address = IPAddrSend
    mmZeroInit(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family         = AF_INET;
//    sin1.sin_len            = sizeof(sin1);
    sin1.sin_addr.s_addr    = inet_addr(REMOTE_IPADDR_STRING);
    sin1.sin_port           = NDK_htons(UDP_CLIENT_PORT);

    /* Connect socket */
    if(connect(sudp, (struct sockaddr *) &sin1, sizeof(sin1) ) < 0) {
        printf("Failed to connect to the socket, %d\n", fdError());
        goto leave;
    }

    /* Allocate a working buffer */
    if(!(pBuf = (char *)malloc(1024))) {
        printf("Failed temp buffer allocation\n");
        goto leave;
    }

    frames_count = TEST_DATA_SIZE * MB / UDP_SERVER_BUFSIZE;

    printf( "    Sending %d frames of %d bytes \n", frames_count, UDP_SERVER_BUFSIZE);

    /* Get the start time */
    llEnter();
    ts = llTimerGetTime(&tsMsec);
    llExit();

    /* Send out all the packets. */
    for(count = 0; count < frames_count; count++) {
        bytes = send(sudp, pBuf, UDP_SERVER_BUFSIZE, 0);
        if(bytes < 0) {
            printf("Error: Send failed Packets Transmitted: %d Error:%d\n", count, fdError());
            goto leave;
        }

        /* Account for the number of bytes txed. */
        totalBytes = totalBytes + bytes;
    }

    /* Get the end time */
    llEnter();
    ts1 = llTimerGetTime(&ts1Msec);
    llExit();

    /* Compute total time in milliseconds */
    startMsec  = (ts * 1000) + tsMsec;
    endMsec    = (ts1 * 1000) + ts1Msec;
    tn = endMsec - startMsec;

    /* Print out the Transmit Results. */
    printf("------------------------------------------------\n");
    printf("Total Data transmitted: %d bytes\n", totalBytes);
    printf("Start Time            : %d msec\n", startMsec);
    printf("End   Time            : %d msec\n", endMsec);
    printf("Total Time expired    : %d msec\n", tn);
    mbps = (((totalBytes/tn)*1000)*8)/1000000;
    if (mbps > 1024) mbps = 1024;           /* we cant be faster than 1 Gig */
    printf("Transmit Throughput : %u Mb/s\n", mbps);
    printf("------------------------------------------------\n");

leave:
    if(pBuf) {
        free(pBuf);
    }

    if(sudp != INVALID_SOCKET) {
       fdClose(sudp);
    }

    TaskSleep(2000);
    fdCloseSession(TaskSelf());
    TaskDestroy(TaskSelf());

}
#endif

static void Trans_Worker(void)
{
    int len = sizeof(Net_Works)/sizeof(Net_Works[0]);
    int i;
    pthread_t thread;
    printf("Net Works begain to start\n");
    Task_Handle subtask;
    Error_Block eb;
    Task_Params subthreadParams;
    Error_init(&eb);
    Task_Params_init(&subthreadParams);
    subthreadParams.stackSize = 0x8000;
    /* create a Task, which is StackTest */
    subtask = Task_create((Task_FuncPtr)Net_Works[0].func, &subthreadParams, &eb);
//    for(i = 0; i < len; i++) {
//        printf("%s create.\n",Net_Works[i].name);
//        pthread_create(&thread, NULL,Net_Works[i].func, Net_Works[i].args);
//    }
}

//static void IPC_Worker(void)
//{
//    int i = 0;
//    for(i = 1; i < 8; i++) {
//       pthread_create(NULL, NULL, (void *(*)(void *))Core0IdleTaskIPC, (void *)i);
//   }
//
//}

/* NetworkOpen */
/* This function is called after the configuration has booted */
static void NetworkOpen()
{
    return;
}

/* NetworkClose */
/*
 * This function is called when the network is shutting down,
 * or when it no longer has any IP addresses assigned to it.
 */
static void NetworkClose()
{
    return;
}

/* NetworkIPAddr */
/*
 * This function is called whenever an IP address binding is
 * added or removed from the system.
 */
static void NetworkIPAddr(uint32_t IPAddr, uint32_t IfIdx, uint32_t fAdd)
{
    static uint16_t fAddGroups = 0;
    uint32_t IPTmp;

//    if(fAdd) {
//        printf("Network Added: ");
//    } else {
//        printf("Network Removed: ");
//    }

    /* Print a message */
    IPTmp = ntohl(IPAddr);
    printf("If-%d:%d.%d.%d.%d\n", IfIdx,
            (uint8_t)(IPTmp>>24)&0xFF, (uint8_t)(IPTmp>>16)&0xFF,
            (uint8_t)(IPTmp>>8)&0xFF, (uint8_t)IPTmp&0xFF );

    /* This is a good time to join any multicast group we require */
    if(fAdd && !fAddGroups) {
        fAddGroups = 1;
    }
    (void) TaskCreate(Trans_Worker, "TransWorker", OS_TASKPRIHIGH, 0x1400, 0, 0, 0);
//    (void) TaskCreate(IPC_Worker, "IPCWorker", OS_TASKPRIHIGH, 0x1400, 0, 0, 0);
#ifdef TCP_CLIENT
    (void) TaskCreate(TCP_perform_send, "TCPBenchmarkTX", OS_TASKPRIHIGH, 0x1400, 0, 0, 0 );
#endif
#ifdef TCP_SERVER
    (void) TaskCreate(TCP_perform_receive, "TCPBenchmarkRX", OS_TASKPRIHIGH, 0x1400, 0, 0, 0);
#endif
#ifdef UDP_CLIENT
    (void) TaskCreate(UDP_perform_send, "UDPBenchmarkTX", OS_TASKPRIHIGH, 0x1400, 0, 0, 0 );
#endif
#ifdef UDP_SERVER
    (void) TaskCreate(UDP_perform_receive, "UDPBenchmarkRX", OS_TASKPRIHIGH, 0x1400, 0, 0, 0);
#endif

    return;
}

/* DHCP_reset() */
/*
 * Code to reset DHCP client by removing it from the active config,
 * and then reinstalling it.
 *
 * Called with:
 * IfIdx    set to the interface (1-n) that is using DHCP.
 * fOwnTask set when called on a new task thread (via TaskCreate()).
 */
void DHCP_reset(uint16_t IfIdx, uint16_t fOwnTask)
{
    CI_SERVICE_DHCPC dhcpc;
    void * h;
    int    rc,tmp;
    uint16_t   idx;

    /*
     * If we were called from a newly created task thread, allow
     * the entity that created us to complete
     */
    if(fOwnTask)
        TaskSleep(500);

    /* Find DHCP on the supplied interface */
    for(idx=1; ; idx++) {
        /* Find a DHCP entry */
        rc = CfgGetEntry(0, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT,
                          idx, &h);
        if(rc != 1)
            goto RESET_EXIT;

        /* Get DHCP entry data */
        tmp = sizeof(dhcpc);
        rc = CfgEntryGetData(h, &tmp, (uint8_t *)&dhcpc);

        /* If not the right entry, continue */
        if((rc<=0) || dhcpc.cisargs.IfIdx != IfIdx) {
            CfgEntryDeRef(h);
            h = 0;
            continue;
        }

        /* Remove the current DHCP service */
        CfgRemoveEntry(0, h);

        /* Specify DHCP Service on specified IF */
        mmZeroInit(&dhcpc, sizeof(dhcpc));
        dhcpc.cisargs.Mode = CIS_FLG_IFIDXVALID;
        dhcpc.cisargs.IfIdx = IfIdx;
        dhcpc.cisargs.pCbSrv = &ServiceReport;
        CfgAddEntry(0, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT, 0,
                     sizeof(dhcpc), (uint8_t *)&dhcpc, 0);
        break;
    }

RESET_EXIT:
    /* If we are a function, return, otherwise, call TaskExit() */
    if(fOwnTask)
        TaskExit();
}

static void ServiceReport(uint32_t Item, uint32_t Status, uint32_t Report, void * h)
{
    printf("Service Status: %-9s: %-9s: %-9s: %03d\n",
            TaskName[Item-1], StatusStr[Status],
            ReportStr[Report/256], Report&0xFF);

    /*
     * Example of adding to the DHCP configuration space
     *
     * When using the DHCP client, the client has full control over access
     * to the first 256 entries in the CFGTAG_SYSINFO space.
     *
     * Note that the DHCP client will erase all CFGTAG_SYSINFO tags except
     * CFGITEM_DHCP_HOSTNAME. If the application needs to keep manual
     * entries in the DHCP tag range, then the code to maintain them should
     * be placed here.
     *
     * Here, we want to manually add a DNS server to the configuration, but
     * we can only do it once DHCP has finished its programming.
     */
    if(Item == CFGITEM_SERVICE_DHCPCLIENT &&
        Status == CIS_SRV_STATUS_ENABLED &&
        (Report == (NETTOOLS_STAT_RUNNING|DHCPCODE_IPADD) ||
         Report == (NETTOOLS_STAT_RUNNING|DHCPCODE_IPRENEW))) {

        uint32_t IPTmp;

        /* Manually add the DNS server when specified */
        IPTmp = inet_addr(DNSServer);
        if(IPTmp)
            CfgAddEntry(0, CFGTAG_SYSINFO, CFGITEM_DHCP_DOMAINNAMESERVER,
                         0, sizeof(IPTmp), (uint8_t *)&IPTmp, 0);
    }

    /* Reset DHCP client service on failure */
    if(Item == CFGITEM_SERVICE_DHCPCLIENT && \
        (Report&~0xFF)==NETTOOLS_STAT_FAULT) {
        CI_SERVICE_DHCPC dhcpc;
        int tmp;

        /* Get DHCP entry data (for index to pass to DHCP_reset). */
        tmp = sizeof(dhcpc);
        CfgEntryGetData(h, &tmp, (uint8_t *)&dhcpc);

        /*
         * Create the task to reset DHCP on its designated IF
         * We must use TaskCreate instead of just calling the function as
         * we are in a callback function.
         */
        TaskCreate((void(*)())DHCP_reset, "DHCPreset", OS_TASKPRINORM, 0x1000,
                    dhcpc.cisargs.IfIdx, 1, 0);
    }
}

/**
 * @brief BIOS task function
 *
 * @details get IP address and open http server
 *
 * @param arg0 the first arg
 *
 * @param arg1 the second arg
 *
 * @return NULL
 */
int ndk_init(UArg arg0, UArg arg1)
{
    int               rc;
    void *            hCfg;
    CI_SERVICE_TELNET telnet;

    /*
     * THIS MUST BE THE ABSOLUTE FIRST THING DONE IN AN APPLICATION before
     *  using the stack!!
     */
    rc = NC_SystemOpen(NC_PRIORITY_LOW, NC_OPMODE_INTERRUPT);
    if(rc) {
        printf("NC_SystemOpen Failed (%d)\n", rc);
        goto task_exit;
    }

    /* Create and build the system configuration from scratch. */
    /* Create a new configuration */
    hCfg = CfgNew();
    if(!hCfg) {
        printf("Unable to create configuration\n");
        goto task_exit;
    }

    /* We better validate the length of the supplied names */
    if(strlen(DomainName) >= CFG_DOMAIN_MAX ||
       strlen(HostName) >= CFG_HOSTNAME_MAX) {
        printf("Names too long\n");
        goto task_exit;
    }

    /* Add our global hostname to hCfg (to be claimed in all connected domains) */
    CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_HOSTNAME, 0,
                 strlen(HostName), (uint8_t *)HostName, 0);

    /* if LocalIPAddr set to 0.0.0.0, use dhcp to get ip address */
    if(inet_addr(LocalIPAddr)) {
        CI_IPNET NA;
        CI_ROUTE RT;
        uint32_t IPTmp;

        /* Setup manual IP address */
        mmZeroInit(&NA, sizeof(NA));
        NA.IPAddr = inet_addr(LocalIPAddr);
        NA.IPMask = inet_addr(LocalIPMask);
        strcpy(NA.Domain, DomainName);
        NA.NetType = 0;

        /* Add the address to interface 1 */
        CfgAddEntry(hCfg, CFGTAG_IPNET, 1, 0,
                           sizeof(CI_IPNET), (uint8_t *)&NA, 0);

        /*
         * Add the default gateway. Since it is the default, the
         * destination address and mask are both zero (we go ahead
         * and show the assignment for clarity).
         */
        mmZeroInit(&RT, sizeof(RT));
        RT.IPDestAddr = 0;
        RT.IPDestMask = 0;
        RT.IPGateAddr = inet_addr(GatewayIP);

        /* Add the route */
        CfgAddEntry(hCfg, CFGTAG_ROUTE, 0, 0,
                           sizeof(CI_ROUTE), (uint8_t *)&RT, 0);

        /* Manually add the DNS server when specified */
        IPTmp = inet_addr(DNSServer);
        if(IPTmp)
            CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_DOMAINNAMESERVER,
                         0, sizeof(IPTmp), (uint8_t *)&IPTmp, 0);
    } else {
        CI_SERVICE_DHCPC dhcpc;

        printf("Configuring DHCP client\n");

        /* Specify DHCP Service on IF-1 */
        mmZeroInit(&dhcpc, sizeof(dhcpc));
        dhcpc.cisargs.Mode = CIS_FLG_IFIDXVALID;
        dhcpc.cisargs.IfIdx = 1;
        dhcpc.cisargs.pCbSrv = &ServiceReport;
        dhcpc.param.pOptions = DHCP_OPTIONS;
        dhcpc.param.len = 2;

        CfgAddEntry(hCfg, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT, 0,
                     sizeof(dhcpc), (uint8_t *)&dhcpc, 0);
    }

    /* Specify TELNET service for our Console example */
    mmZeroInit(&telnet, sizeof(telnet));
    telnet.cisargs.IPAddr = INADDR_ANY;
    telnet.cisargs.pCbSrv = &ServiceReport;
    telnet.param.MaxCon = 2;
    telnet.param.Callback = &ConsoleOpen;
    CfgAddEntry(hCfg, CFGTAG_SERVICE, CFGITEM_SERVICE_TELNET, 0,
                 sizeof(telnet), (uint8_t *)&telnet, 0);

    /*
     * This code sets up the TCP and UDP buffer sizes
     * (Note 8192 is actually the default. This code is here to
     * illustrate how the buffer and limit sizes are configured.)
     */

    /* TCP Transmit buffer size */
    rc = 64000;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPTXBUF,
                 CFG_ADDMODE_UNIQUE, sizeof(uint16_t), (uint8_t *)&rc, 0);

    /* TCP Receive buffer size (copy mode) */
    rc = 64000;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXBUF,
                 CFG_ADDMODE_UNIQUE, sizeof(uint16_t), (uint8_t *)&rc, 0);

    /* TCP Receive limit (non-copy mode) */
    rc = 64000;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXLIMIT,
                 CFG_ADDMODE_UNIQUE, sizeof(uint16_t), (uint8_t *)&rc, 0);

    /* UDP Receive limit */
    rc = 64000;
    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKUDPRXLIMIT,
                 CFG_ADDMODE_UNIQUE, sizeof(uint16_t), (uint8_t *)&rc, 0);
    /*
     * Boot the system using this configuration
     *
     * We keep booting until the function returns 0. This allows
     * us to have a "reboot" command.
     */
    do {
        rc = NC_NetStart(hCfg, NetworkOpen, NetworkClose, NetworkIPAddr);
    } while(rc > 0);

    /* Delete Configuration*/
    CfgFree(hCfg);

    /* Close the OS */
task_exit:
    NC_SystemClose();
    Task_exit();
    return 0;
}
