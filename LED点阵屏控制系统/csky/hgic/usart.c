
#include "typesdef.h"
#include "list.h"
#include "dev.h"
#include "devid.h"
#include "osal/string.h"
#include "hal/uart.h"

#include "csi_core.h"
#include "drv_usart.h"

static int32_t _disable_printf_ = 0;
int32_t csi_usart_getchar(usart_handle_t handle, uint8_t *ch)
{
    *ch = uart_getc((struct uart_device *)handle);
    return RET_OK;
}

int32_t csi_usart_putchar(usart_handle_t handle, uint8_t ch)
{
    if (_disable_printf_) { return 0; }
    return uart_putc((struct uart_device *)handle, ch);
}

void disable_print(void)
{
    _disable_printf_ = 1;
}

void enable_print(void)
{
    _disable_printf_ = 0;
}

