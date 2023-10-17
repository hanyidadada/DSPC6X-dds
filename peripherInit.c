/*
 * peripherInit.c
 *
 *  Created on: 2023年9月22日
 *      Author: hanyi
 */

#include <stdio.h>

#include <xdc/runtime/Error.h>

/* csl include */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/cslr_uart.h>
#include <ti/csl/csl_gpioAux.h>

#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>
/* USER include */
#include "system/platform.h"
#include "driver/c66x_uart.h"
#include "core/net_trans.h"

/* global variable */
uint8_t rx_flag = 0;
uint8_t uartbuf[100] = {0};
uint32_t index = 0;
uint8_t err_flag = 0;
int8_t err_type = 0;

/* Interrupt service function */
void uart_isr(void *arg)
{
    uint32_t int_id = 0;
    uint8_t rx_data;

    int_id = uart_int_stat(CSL_UART_REGS);

    /* recv interrupt */
    if(int_id == CSL_UART_IIR_INITD_RDA) {
        rx_flag = 1;
        if(uart_isdata_ready(CSL_UART_REGS)) {
            rx_data = uart_read_data(CSL_UART_REGS);
            uartbuf[index] = rx_data;
            index++;
        }
    }

    /* error interrupt */
    if(int_id == CSL_UART_IIR_INITD_RLS) {
        err_flag = 1;
        err_type = uart_error_get(CSL_UART_REGS);
    }

    /* clear the system interrupt in the interrupt controller */

    Hwi_clearInterrupt(13);
    CpIntc_clearSysInt(0,  *(int*)arg);
}


void uart_interrupt_init(void)
{
    Hwi_Params hwiParams;
    Int eventId;
    Hwi_Handle uartHwi;
    Error_Block eb;
    uart_interrupt_enable();
    // Map system interrupt CSL_INTC0_UARTINT to host interrupt 8 on Intc 0
    CpIntc_mapSysIntToHostInt(0, CSL_INTC0_UARTINT, 8);

    // Plug the function and argument for System interrupt CSL_INTC0_UARTINT then enable it
    CpIntc_dispatchPlug(CSL_INTC0_UARTINT, (CpIntc_FuncPtr)uart_isr, CSL_INTC0_UARTINT, TRUE);

    // Enable Host interrupt 8 on Intc 0
    CpIntc_enableHostInt(0, 8);
    CpIntc_enableSysInt(0, CSL_INTC0_UARTINT);

    // Get the eventId associated with Host interrupt 8
    eventId = CpIntc_getEventId(8);
    // 使用默认值初始化参数
    Hwi_Params_init(&hwiParams);
    // 中断事件
    hwiParams.eventId = eventId;
    // 传递到中断服务函数的参数
    hwiParams.arg = 8;

    hwiParams.maskSetting = Hwi_MaskingOption_SELF;
    // 使能中断
    hwiParams.enableInt = TRUE;
    // 可屏蔽中断 13
    uartHwi = Hwi_create(13, &CpIntc_dispatch, &hwiParams, &eb);
    if (uartHwi == NULL) {
        printf("Error create Hwi\n");
    }
    printf("Hwi init succ\n");
}

int PeripherInit(void)
{
    uint32_t main_pll_freq;
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
//    CSL_BootCfgLockKicker();

    uart_init(CSL_UART_REGS);
    /* Get the cpu freq */
    main_pll_freq = platform_get_main_pll_freq();

    /* Set the default baud rate to 115200 */
    /*
     * CPU frequency = main pll out
     * uart input clock = main pll out / 6
     */
    uart_set_baudrate(CSL_UART_REGS, main_pll_freq/6, 115200);
    uart_interrupt_init();
    return 0;
}
