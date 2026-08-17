/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (2)
#endif
/* ISR prototypes */
void ipc_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_IPC_IRQ0 ((IRQn_Type) 0) /* IPC IRQ0 (CPU Mutual Interrupt 0) */
#define IPC_IRQ0_IRQn          ((IRQn_Type) 0) /* IPC IRQ0 (CPU Mutual Interrupt 0) */
#define VECTOR_NUMBER_IPC_IRQ1 ((IRQn_Type) 1) /* IPC IRQ1 (CPU Mutual Interrupt 1) */
#define IPC_IRQ1_IRQn          ((IRQn_Type) 1) /* IPC IRQ1 (CPU Mutual Interrupt 1) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (2)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
