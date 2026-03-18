#include "typesdef.h"
#include "osal/string.h"
#include "csi_core.h"

#ifndef ASSERT_HOLDUP
#define ASSERT_HOLDUP 0
#endif

uint8_t assert_holdup = ASSERT_HOLDUP;
void assert_internal(const char *__function, unsigned int __line, const char *__assertion)
{
    int i = 0;

    if (assert_holdup) {
        mcu_disable_watchdog();
        jtag_map_set(1);
    }

    do {
        printf("assertation \"%s\" failed: function: %s, line %d\r\n", __assertion, __function, __line);
        for (i = 0; i < 20000000; i++);
    } while (assert_holdup);
}

