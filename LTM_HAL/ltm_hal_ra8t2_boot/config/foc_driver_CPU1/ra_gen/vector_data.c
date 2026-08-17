/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = ipc_isr, /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [1] = ipc_isr, /* IPC IRQ1 (CPU Mutual Interrupt 1) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_IPC_IRQ0,GROUP0), /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_IPC_IRQ1,GROUP1), /* IPC IRQ1 (CPU Mutual Interrupt 1) */
        };
        #endif
        #endif
