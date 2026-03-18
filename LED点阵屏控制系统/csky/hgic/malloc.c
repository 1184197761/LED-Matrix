#include "sys_config.h"
#include "typesdef.h"
#include "list.h"
#include "dev.h"
#include "devid.h"
#include "osal/string.h"
#include "hal/uart.h"
#include "lib/heap/sysheap.h"

#include "csi_core.h"
#include "drv_usart.h"

#ifdef MPOOL_ALLOC

void *krhino_mm_alloc(size_t size, void *caller)
{
#ifdef MEM_TRACE
    return _os_malloc_t(size, caller, 0);
#else
    return _os_malloc(size);
#endif
}

void krhino_mm_free(void *ptr)
{
    os_free(ptr);
}

void k_mm_init(void)
{
    uint32 flags = 0;
#ifdef MEM_TRACE
    flags |= SYSHEAP_FLAGS_MEM_LEAK_TRACE | SYSHEAP_FLAGS_MEM_OVERFLOW_CHECK;
#endif
    sysheap_init((void *)SYS_HEAP_START, SYS_HEAP_SIZE, flags);
}

#endif

