/*
 * MultiBoot.c
 *
 *  Created on: 2021-11-16
 *      Author: 27242
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ti/platform/platform.h>
#include <ti/platform/resource_mgr.h>
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/_stack.h>
#include <ti/ndk/inc/tools/console.h>
#include <ti/ndk/inc/tools/servers.h>
#include <ti/csl/csl_ipcAux.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_pscAux.h>
/* Platform utilities include */
#include "system/platform.h"
#include "driver/c66x_uart.h"
#include "multiBoot.h"
unsigned char * g_restructStartCmd = (unsigned char *) RESTRUCT_START_CMD;

void CoreStart(unsigned int EntryAddr, int corenum)
{
    unsigned int MAGIC_ADDR = 0X1087FFFC + corenum * 0x1000000;
    CSL_BootCfgUnlockKicker();
    *((volatile unsigned int *)MAGIC_ADDR) = EntryAddr;
    CSL_IPC_genGEMInterrupt (corenum, 0);
//    CSL_BootCfgLockKicker();
    return;
}

void Load_Core_app_Start(unsigned int EntryAddr, int corenum)
{
    CSL_PSC_setModuleLocalReset(corenum + 15, PSC_MDLRST_ASSERTED);
    CSL_PSC_setModuleLocalReset(corenum + 15, PSC_MDLRST_DEASSERTED);
    TaskSleep(100);

    CoreStart(EntryAddr, corenum);
    return;
}


static inline unsigned int strtohex(unsigned char *str, int len)
{
    int i =0;
    unsigned char temp;
    unsigned  int result = 0;
    for(i = 0; i<len; i++) {
        temp = toupper(str[i]) - 0x30;
        if (temp > 9) {
            temp -= 7;
        }
        result = (result << 4) + temp;
    }
    return result;
}

// void LoadTask(UINT32 handle)
// {
//     struct timeval  to;
//     fdOpenSession(TaskSelf());
//     SOCKET connfd = (SOCKET)handle;
//     unsigned int dataNum = 0;
//     volatile unsigned int Entry_Address=0;
//     volatile unsigned char* WriteAddr = 0;
//     unsigned int dspNum;
//     unsigned char pBuf[4];
//     unsigned char data;
//     unsigned char dataGroupNum;
//     struct ARGS_S args;
//     SendCmd sendCmd = {0};
//     int corenum = 1;
//     int32_t bytes = 0;
//     to.tv_sec  = 60;
//     to.tv_usec = 0;
//     setsockopt(connfd, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
//     setsockopt(connfd, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));

//     memset(&args, 0, sizeof(struct ARGS_S));
//     bytes = (int)recv(connfd, (void *)&args, sizeof(struct ARGS_S), 0);
//     if(bytes != sizeof(struct ARGS_S)) {
//         printf("File Head Error 1!bytes: %d!\n", bytes);
//         goto leave;
//     }
//     //Entry_Address = strtohex(args.Entry_adress, 8);
//     //WriteAddr = (unsigned int*)strtohex(args.WriteAddr, 8);
//     //dspNum = args.dspNum - 0x30;
//     //corenum = args.corenum - 0x30;
//     Entry_Address = args.Entry_adress;
//     WriteAddr = (unsigned char*)args.WriteAddr;
//     dspNum = args.dspNum;
//     corenum = args.corenum;

//     printf("%x %x %d\n", Entry_Address, WriteAddr, corenum);
//     CSL_PSC_setModuleLocalReset(corenum + 15, PSC_MDLRST_ASSERTED); 
//     CSL_PSC_setModuleLocalReset(corenum + 15, PSC_MDLRST_DEASSERTED); 

//     while (1) {
//         //memset(pBuf, 0, 4);
//         bytes = recv(connfd, &data, 1, 0);
        
//         //printf("bytes num = 0x%x \r\n", bytes);
//         if (bytes <= 0) {
//             break;
//         }
//         dataNum++;
//         *WriteAddr = data;

//         WriteAddr++;
//     }

//     if (dspNum != 1) {
//         sendCmd.syncHeader = SYNC_HEADER;
//         sendCmd.deviceID = DEVICE_CODE_CMD_CROSS_DEVICE;
//         sendCmd.cmdLength = htons(0x08);
//         sendCmd.deviceFlag = 0x01;
//         sendCmd.funcFlag = FUNC_FLAG_DSP_CONTROL;
//         sendCmd.coreNum = corenum;
//         sendCmd.cmdType = CMD_TYPE_RESTRUCT_BEGIN;
//         sendCmd.entranceAddress = htonl(Entry_Address);

//         memcpy(g_restructStartCmd, &sendCmd, sizeof(SendCmd));

//         dataGroupNum = (dataNum % 4 == 0) ? (dataNum / 4) : ((dataNum / 4) + 1);
        
//         /* Send restruct data */
//         send_CSL(0, (unsigned int)args.WriteAddr, (unsigned int)args.WriteAddr, 4 * dataGroupNum, C6678_B_ID, 0);
        
//         /* Send restruct command */
//         send_CSL(0, (unsigned int)g_restructStartCmd, (unsigned int)g_restructStartCmd, sizeof(SendCmd), C6678_B_ID, 0);
        
//         return;
//     }

//     printf("dataNum 0x%x \r\n", dataNum);
//     CACHE_wbAllL2(CACHE_WAIT);
//     CACHE_wbAllL1d(CACHE_WAIT);
//     Load_Core_app_Start(Entry_Address, corenum);
//     printf("Restart core 0x%x \r\n", corenum);
// leave:
//     //if(connfd != INVALID_SOCKET) {
//     //    fdClose(connfd);
//     //}
//     //TaskSleep(2000);
//     //fdCloseSession(TaskSelf());
//     //TaskDestroy(TaskSelf());

// }

// void GetImage(UINT32 handle)
// {
//     struct timeval  to;
//     fdOpenSession(TaskSelf());
//     SOCKET connfd = (SOCKET)handle;
//     volatile unsigned int* flag = (unsigned int *)0x0C000000;
//     volatile uint8_t *IMAGEADDR = (uint8_t*)0x0C000004;
//     uint32_t length = 173878;
//     unsigned char pBuf[4];

//     if(*flag != 1) {
//         strcpy(pBuf,"fail");
//         send(connfd, pBuf, 4, 0);
//         goto leave;
//     }
//     strcpy(pBuf,"succ");
//     send(connfd, pBuf, 4, 0);
//     send(connfd, (void*)IMAGEADDR, 173878, 0);
//     uart_printf("send ok!");
// leave:
//     if(connfd != INVALID_SOCKET) {
//         fdClose(connfd);
//     }
//     TaskSleep(2000);
//     fdCloseSession(TaskSelf());
//     TaskDestroy(TaskSelf());

// }


void ResetCore(u32 corenum)
{
    printf("Core num = %d\r\n", corenum);
    CSL_PSC_setModuleLocalReset(corenum + 15, PSC_MDLRST_ASSERTED);
    CSL_PSC_setModuleLocalReset(corenum + 15, PSC_MDLRST_DEASSERTED);
    printf("reset ok\n");
    return;
}

