/**
 * Copyright (C) 2013 Guangzhou Tronlong Electronic Technology Co., Ltd. - www.tronlong.com
 *
 * @file platform_osal.c
 *
 * @brief This file implements the OSAL layers for the various LLD's and libraries
 * that the application use.
 *
 * @author Tronlong <support@tronlong.com>
 *
 * @version V1.0
 *
 * @date 2020-08-09
 *
 **/

/* C standard Header files */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* XDC/BIOS includes */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

/* BIOS6 include */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

/* CSL CHIP, SEM Functional layer includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
/* CSL Cache module includes */
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/tistdtypes.h>

#include <ti/platform/platform.h>

/* Platform utilities include */
#include "system/resource_mgr.h"

#include "nimu_eth.h"

/**********************************************************************
 *********************** Platform Library OSAL Functions **************
 **********************************************************************/
/**
 * ============================================================================
 *  @n@b Osal_platformMalloc
 *
 *  @b  brief
 *  @n  This routine implements the memory handler for Platform Library.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
uint8_t *Osal_platformMalloc (uint32_t num_bytes, uint32_t alignment)
{
    Error_Block     errorBlock;

    /* Allocate memory.  */
    Error_init(&errorBlock);

    return Memory_alloc(NULL, num_bytes, alignment, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_platformFree
 *
 *  @b  brief
 *  @n  This routine implements the memory free handler for Platform Library.
 *      Frees up memory allocated using
 *      @a Osal_platformMalloc ()
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
void Osal_platformFree (uint8_t *dataPtr, uint32_t num_bytes)
{

    /* Free up the memory */
    if (dataPtr)
    {
        Memory_free(NULL, dataPtr, num_bytes);
    }
}

/**
 * ============================================================================
 *  @n@b Osal_platformSpiCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization for the SPI bus.
 *
 *      This is a BLOCKING API.
 *.
 *
 *  @param[in]
 *  @n  None
 *
 *  @return
 *  @n  Nothing
 * =============================================================================
 */
void Osal_platformSpiCsEnter(void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (PLATFORM_SPI_HW_SEM)) == 0);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_platformSpiCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_platformSpiCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab the SPI bus.
 *
 *
 *  @return     None
 * =============================================================================
 */
void Osal_platformSpiCsExit (void)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (PLATFORM_SPI_HW_SEM);

    return;
}


/**********************************************************************
 *********************** NIMU OSAL Functions **************************
 **********************************************************************/
/**
 * ============================================================================
 *  @n@b Osal_nimuMalloc
 *
 *  @b  brief
 *  @n  This routine implements the memory handler for the NIMU library. The API
 *      allocates a memory block of a given size specified by input parameter 'num_bytes'.
 *
 *      This API should allocate memory from shared memory.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
Ptr Osal_nimuMalloc (uint32_t num_bytes, uint32_t alignment)
{
    Error_Block     errorBlock;

    /* Allocate memory.  */
    Error_init(&errorBlock);

    return Memory_alloc(NULL, num_bytes, alignment, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_nimuFree
 *
 *  @b  brief
 *  @n  This API frees and restores a given memory location
 *      pointer 'dataPtr' of size 'num_bytes' to its
 *      original heap location. Frees up memory allocated using
 *      @a Osal_nimuMalloc ()
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
void Osal_nimuFree (Ptr dataPtr, uint32_t num_bytes)
{

    /* Free up the memory */
    if (dataPtr)
    {
        Memory_free(NULL, dataPtr, num_bytes);
    }
}

/**********************************************************************
 *********************** CPPI OSAL Functions **************************
 **********************************************************************/

/**
 * ============================================================================
 *  @n@b Osal_cppiCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access CPPI shared
 *      library at the same time.
 *
 *  @param[in]
 *  @n  None
 *
 *  @return
 *  @n  Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_cppiCsEnter (void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (PLATFORM_CPPI_HW_SEM)) == 0);

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_cppiCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_cppiCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab CPPI access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
void Osal_cppiCsExit (Ptr CsHandle)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (PLATFORM_CPPI_HW_SEM);

    return;
}


/**
 * ============================================================================
 *  @n@b Osal_cppiMalloc
 *
 *  @b  brief
 *  @n  This API allocates a memory block of a given
 *      size specified by input parameter 'num_bytes'.
 *
 *      This API should allocate memory from shared memory if the test applications
 *      are to be run on multiple cores.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
Ptr Osal_cppiMalloc (uint32_t num_bytes)
{
    Error_Block     errorBlock;

    /* Allocate memory.  */
    Error_init(&errorBlock);

    return Memory_alloc(NULL, num_bytes, PLATFORM_CACHE_LINE_SIZE, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_cppiFree
 *
 *  @b  brief
 *  @n  This API frees and restores a given memory location
 *      pointer 'dataPtr' of size 'num_bytes' to its
 *      original heap location. Frees up memory allocated using
 *      @a Osal_cppiMalloc ()
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
void Osal_cppiFree (Ptr dataPtr, uint32_t num_bytes)
{

    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
        Memory_free(NULL, dataPtr, num_bytes);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be invalidated
 *
 *  @param[in]  size
 *       Size of the block to be invalidated

 *  @retval
 *      Not Applicable
 */
void Osal_cppiBeginMemAccess (void *blockPtr, uint32_t size)
{
    /* Cache coherence protection is only required for multi-core application */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be written back
 *
 *  @param[in]  size
 *       Size of the block to be written back

 *  @retval
 *      Not Applicable
 */
void Osal_cppiEndMemAccess (void *blockPtr, uint32_t size)
{
    /* Cache coherence protection is only required for multi-core application */
    return;
}

/**********************************************************************
 *********************** QMSS OSAL Functions **************************
 **********************************************************************/

/**
 * ============================================================================
 *  @n@b Osal_qmssCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access QMSS shared
 *      library at the same time.
 *
 *  @param[in]  None
 *
 *  @return
 *      Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_qmssCsEnter (void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core QMSS synchronization lock
     */
    while ((CSL_semAcquireDirect (PLATFORM_QMSS_HW_SEM)) == 0);

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_cpswQmssCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab QMSS access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
void Osal_qmssCsExit (Ptr CsHandle)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (PLATFORM_QMSS_HW_SEM);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMtCsEnter
 *
 *  @b  brief
 *  @n  This API ensures ONLY multi-threaded
 *      synchronization to the QMSS user.
 *
 *      This is a BLOCKING API.
 *
 *  @param[in] None
 *
 *  @return
 *       Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_qmssMtCsEnter (void)
{
    /*
     * Current demos are not multi-threaded.
     */
    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMtCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_cpswQmssMtCsEnter ()
 *      API. It resets the multi-threaded lock, enabling another process
 *      on the current core to grab it.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
void Osal_qmssMtCsExit (Ptr CsHandle)
{
    /*
     * Release multi-threaded / multi-process lock on this core.
     */
    return;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMalloc
 *
 *  @b  brief
 *  @n  This API allocates a memory block of a given
 *      size specified by input parameter 'num_bytes'.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
Ptr Osal_qmssMalloc (uint32_t num_bytes)
{
    Error_Block     errorBlock;

    /* Allocate memory.  */
    Error_init(&errorBlock);

    return Memory_alloc(NULL, num_bytes, PLATFORM_CACHE_LINE_SIZE, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_qmssFree
 *
 *  @b  brief
 *  @n  This API frees and restores a given memory location
 *      pointer 'dataPtr' of size 'num_bytes' to its
 *      original heap location. Frees up memory allocated using
 *      @a Osal_qmssMalloc ()
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
void Osal_qmssFree (Ptr dataPtr, uint32_t num_bytes)
{

    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
        Memory_free(NULL, dataPtr, num_bytes);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be invalidated
 *
 *  @param[in]  size
 *       Size of the block to be invalidated

 *  @retval
 *      Not Applicable
 */
void Osal_qmssBeginMemAccess (void *blockPtr, uint32_t size)
{

    /* Cache coherence protection is only required for multi-core application */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be written back
 *
 *  @param[in]  size
 *       Size of the block to be written back

 *  @retval
 *      Not Applicable
 */
void Osal_qmssEndMemAccess (void *blockPtr, uint32_t size)
{

    /* Cache coherence protection is only required for multi-core application */
    return;
}


/**********************************************************************
 *********************** PASS OSAL Functions **************************
 **********************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be invalidated
 *
 *  @param[in]  size
 *       Size of the block to be invalidated

 *  @retval
 *      Not Applicable
 */
void Osal_paBeginMemAccess (Ptr addr, uint32_t size)
{
    /* Cache coherence protection is only required for multi-core application */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  addr
 *       Address of the block which is to be written back
 *
 *  @param[in]  size
 *       Size of the block to be written back

 *  @retval
 *      Not Applicable
 */
void Osal_paEndMemAccess (Ptr addr, uint32_t size)
{
    /* Cache coherence protection is only required for multi-core application */
    return;
}


/* OSAL Functions for the PA LLDs */
Void Osal_paMtCsEnter (uint32_t *key)
{
}

Void Osal_paMtCsExit (uint32_t key)
{
}

Void* Osal_qmssAccCsEnter (Void)
{
//    /* Get the hardware semaphore.
//     *
//     * Acquire Multi core QMSS synchronization lock
//     */
//    while ((CSL_semAcquireDirect (QMSS_ACC_HW_SEM)) == 0);
//
//    /* Disable all interrupts and OS scheduler.
//     *
//     * Acquire Multi threaded / process synchronization lock.
//     */
//    coreKey [CSL_chipReadDNUM ()] = Hwi_disable();

    return NULL;
}

Void Osal_qmssAccCsExit (Void *CsHandle)
{
//    /* Enable all interrupts and enables the OS scheduler back on.
//     *
//     * Release multi-threaded / multi-process lock on this core.
//     */
//    Hwi_restore(coreKey [CSL_chipReadDNUM ()]);
//
//    /* Release the hardware semaphore
//     *
//     * Release multi-core lock.
//     */
//    CSL_semReleaseSemaphore (QMSS_ACC_HW_SEM);

    return;
}


#ifdef C665_PLATFORMS
/**********************************************************************
 *********************** EMAC OSAL Functions **************************
 **********************************************************************/

 /**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  addr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block in bytes
 *
 *  @retval
 *      None
 */
void
Osal_emacBeginMemAccess
(
    void*   addr,
    Uint32  size
)
{
    (void)addr;
    (void)size;

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  addr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block in bytes
 *
 *  @retval
 *      None
 */
void
Osal_emacEndMemAccess
(
    void*   addr,
    Uint32  size
)
{
    (void)addr;
    (void)size;

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to provide critical section to prevent access of shared
 *      resources from single core and multiple threads.
 *
 *  @param[in]  port_num
 *      EMAC port number which needs critical section to protect its resources.
 *
 *  @retval
 *      None
 */
void
Osal_emacEnterSingleCoreCriticalSection
(
    Uint32      port_num
)
{
    if (port_num == 0)
    {
        IER &= ~NIMU_EMAC0_INT_FLAG;
    }
    else
    {
        IER &= ~NIMU_EMAC1_INT_FLAG;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is called to end the critical section access of shared resources
 *      from single cores.
 *
 *  @param[in]  port_num
 *      EMAC port number which needs critical section to protect its resources.
 *
 *  @retval
 *      None
 */
void
Osal_emacExitSingleCoreCriticalSection
(
    Uint32      port_num
)
{
    if (port_num == 0)
    {
        IER |= NIMU_EMAC0_INT_FLAG;
    }
    else
    {
        IER |= NIMU_EMAC1_INT_FLAG;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to provide critical section to prevent access of shared
 *      resources from multiple cores
 *
 *  @param[in]  port_num
 *      EMAC port number which needs critical section to protect its resources.
 *
 *  @retval
 *      None
 */
void
Osal_emacEnterMultipleCoreCriticalSection
(
    Uint32      port_num
)
{
    (void)port_num;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called to end the critical section access of shared resources
 *      from multiple cores.
 *
 *  @param[in]  port_num
 *      EMAC port number which needs critical section to protect its resources.
 *
 *  @retval
 *      None
 */
void
Osal_emacExitMultipleCoreCriticalSection
(
    Uint32      port_num
)
{
    (void)port_num;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a memory block of the specified size.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
void* Osal_emacMalloc
(
    Uint32  num_bytes
)
{
    return mmAlloc(num_bytes);
}

/**
 *  @b Description
 *  @n
 *      The function is used to clean up a specific memory block
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *  @param[in]  num_bytes
 *      Size of the memory block being cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
void
Osal_emacFree
(
    void*       ptr,
    Uint32      num_bytes
)
{
    (void)num_bytes;
    mmFree(ptr);
}
#endif
