/*
 * main.c
 *
 *  Created on: 2023��9��21��
 *      Author: hanyi
 */

/* C standard Header files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>

/* BIOS6 include */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
// c66 hwi module is compatible with c64p


#include "peripherInit.h"
#include "core/net_trans.h"
/**
 * @brief main function
 *
 * @details Program unique entry
 *
 * @param void
 *
 * @return successful execution of the program
 *     @retval 0 successful
 *     @retval 1 failed
 */

int main(void)
{
    Task_Handle ndktask;
    Error_Block eb;
    Task_Params ndkTaskParams;

    if(PeripherInit() != 0) {
        printf("PeripherInit error!\n");
        return -1;
    }

//    InitIpc();
    /* Variable initialization */
    Error_init(&eb);

    /* Task params initialization */
    Task_Params_init(&ndkTaskParams);
    ndkTaskParams.stackSize = 0x8000;
    /* create a Task, which is StackTest */
    ndktask = Task_create((Task_FuncPtr)ndk_init, &ndkTaskParams, &eb);
    if(ndktask == NULL) {
        printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    /* Start the BIOS 6 Scheduler */
    BIOS_start();

    return 0;
}
