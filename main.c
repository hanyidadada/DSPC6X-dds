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
    Task_Handle task;
    Error_Block eb;
    Task_Params TaskParams;
//    char tips_strings[] = {"\r\ntl-uart-echo Application\r\n"};

    if(PeripherInit() != 0) {
        printf("PeripherInit error!\n");
        return -1;
    }

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
