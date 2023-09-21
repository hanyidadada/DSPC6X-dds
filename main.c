/*
 * main.c
 *
 *  Created on: 2023Äê9ÔÂ21ÈÕ
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

/* csl include */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_cacheAux.h>

/* USER include */
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
    Task_Handle task;
    Error_Block eb;
    Task_Params TaskParams;

    /*
     * Set L2 cache size as 64KB
     * equivalent to the l2Mode configuration in the Platform.xdc file
     */
    CACHE_setL2Size(CACHE_64KCACHE);

    /* Start TCSL so its free running */
    CSL_chipWriteTSCL(0);

    /* Enable the PSC */
    psc_init();

    /* Unlock the chip configuration registers */
    CSL_BootCfgUnlockKicker();

    /* initialize the sgmii network subsystem */
    init_sgmii(0);
    init_sgmii(1);

    if(queue_manager_init() != 0)
        return -1;
    /* Lock the chip configuration registers */
    CSL_BootCfgLockKicker();

    /* Variable initialization */
    Error_init(&eb);

    /* Task params initialization */
    Task_Params_init(&TaskParams);

    TaskParams.stackSize = 0x8000;

    /* create a Task, which is StackTest */
    task = Task_create((Task_FuncPtr)ndk_init, &TaskParams, &eb);
    if(task == NULL) {
        printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    /* Start the BIOS 6 Scheduler */
    BIOS_start();

    return 0;
}
