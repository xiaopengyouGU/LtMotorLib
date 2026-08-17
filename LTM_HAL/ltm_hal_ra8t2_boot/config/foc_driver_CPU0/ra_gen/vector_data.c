/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = adc_b_calend0_isr, /* ADC CALEND0 (End of calibration of A/D converter unit 0) */
            [1] = adc_b_calend1_isr, /* ADC CALEND1 (End of calibration of A/D converter unit 1) */
            [2] = adc_b_adi0_isr, /* ADC ADI0 (End of A/D scanning operation(Gr.0)) */
            [3] = adc_b_adi1_isr, /* ADC ADI1 (End of A/D scanning operation(Gr.1)) */
            [4] = spi_b_tei_isr, /* SPI0 TEI (Transmission complete event) */
            [5] = spi_b_eri_isr, /* SPI0 ERI (Error) */
            [6] = dmac_int_isr, /* DMAC2 INT (DMAC2 transfer end) */
            [7] = dmac_int_isr, /* DMAC3 INT (DMAC3 transfer end) */
            [8] = spi_b_tei_isr, /* SPI1 TEI (Transmission complete event) */
            [9] = spi_b_eri_isr, /* SPI1 ERI (Error) */
            [10] = dmac_int_isr, /* DMAC0 INT (DMAC0 transfer end) */
            [11] = dmac_int_isr, /* DMAC1 INT (DMAC1 transfer end) */
            [12] = ipc_isr, /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [13] = ipc_isr, /* IPC IRQ1 (CPU Mutual Interrupt 1) */
            [14] = sci_b_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [15] = sci_b_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [16] = sci_b_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [17] = sci_b_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [18] = canfd_error_isr, /* CAN1 CHERR (Channel  error) */
            [19] = canfd_channel_tx_isr, /* CAN1 TX (Transmit interrupt) */
            [20] = canfd_common_fifo_rx_isr, /* CAN1 COMFRX (Common FIFO receive interrupt) */
            [21] = canfd_error_isr, /* CAN GLERR (Global error) */
            [22] = canfd_rx_fifo_isr, /* CAN RXF (Global receive FIFO interrupt) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_ADC_CALEND0,GROUP0), /* ADC CALEND0 (End of calibration of A/D converter unit 0) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_ADC_CALEND1,GROUP1), /* ADC CALEND1 (End of calibration of A/D converter unit 1) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_ADC_ADI0,GROUP2), /* ADC ADI0 (End of A/D scanning operation(Gr.0)) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_ADC_ADI1,GROUP3), /* ADC ADI1 (End of A/D scanning operation(Gr.1)) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SPI0_TEI,GROUP4), /* SPI0 TEI (Transmission complete event) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SPI0_ERI,GROUP5), /* SPI0 ERI (Error) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_DMAC2_INT,GROUP6), /* DMAC2 INT (DMAC2 transfer end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_DMAC3_INT,GROUP7), /* DMAC3 INT (DMAC3 transfer end) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SPI1_TEI,GROUP0), /* SPI1 TEI (Transmission complete event) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SPI1_ERI,GROUP1), /* SPI1 ERI (Error) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_DMAC0_INT,GROUP2), /* DMAC0 INT (DMAC0 transfer end) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_DMAC1_INT,GROUP3), /* DMAC1 INT (DMAC1 transfer end) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_IPC_IRQ0,GROUP4), /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_IPC_IRQ1,GROUP5), /* IPC IRQ1 (CPU Mutual Interrupt 1) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP6), /* SCI9 RXI (Receive data full) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP7), /* SCI9 TXI (Transmit data empty) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP0), /* SCI9 TEI (Transmit end) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP1), /* SCI9 ERI (Receive error) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_CAN1_CHERR,GROUP2), /* CAN1 CHERR (Channel  error) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_CAN1_TX,GROUP3), /* CAN1 TX (Transmit interrupt) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_CAN1_COMFRX,GROUP4), /* CAN1 COMFRX (Common FIFO receive interrupt) */
            [21] = BSP_PRV_VECT_ENUM(EVENT_CAN_GLERR,GROUP5), /* CAN GLERR (Global error) */
            [22] = BSP_PRV_VECT_ENUM(EVENT_CAN_RXF,GROUP6), /* CAN RXF (Global receive FIFO interrupt) */
        };
        #endif
        #endif
