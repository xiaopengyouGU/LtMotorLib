/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (23)
#endif
/* ISR prototypes */
void adc_b_calend0_isr(void);
void adc_b_calend1_isr(void);
void adc_b_adi0_isr(void);
void adc_b_adi1_isr(void);
void spi_b_tei_isr(void);
void spi_b_eri_isr(void);
void dmac_int_isr(void);
void ipc_isr(void);
void sci_b_uart_rxi_isr(void);
void sci_b_uart_txi_isr(void);
void sci_b_uart_tei_isr(void);
void sci_b_uart_eri_isr(void);
void canfd_error_isr(void);
void canfd_channel_tx_isr(void);
void canfd_common_fifo_rx_isr(void);
void canfd_rx_fifo_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_ADC_CALEND0 ((IRQn_Type) 0) /* ADC CALEND0 (End of calibration of A/D converter unit 0) */
#define ADC_CALEND0_IRQn          ((IRQn_Type) 0) /* ADC CALEND0 (End of calibration of A/D converter unit 0) */
#define VECTOR_NUMBER_ADC_CALEND1 ((IRQn_Type) 1) /* ADC CALEND1 (End of calibration of A/D converter unit 1) */
#define ADC_CALEND1_IRQn          ((IRQn_Type) 1) /* ADC CALEND1 (End of calibration of A/D converter unit 1) */
#define VECTOR_NUMBER_ADC_ADI0 ((IRQn_Type) 2) /* ADC ADI0 (End of A/D scanning operation(Gr.0)) */
#define ADC_ADI0_IRQn          ((IRQn_Type) 2) /* ADC ADI0 (End of A/D scanning operation(Gr.0)) */
#define VECTOR_NUMBER_ADC_ADI1 ((IRQn_Type) 3) /* ADC ADI1 (End of A/D scanning operation(Gr.1)) */
#define ADC_ADI1_IRQn          ((IRQn_Type) 3) /* ADC ADI1 (End of A/D scanning operation(Gr.1)) */
#define VECTOR_NUMBER_SPI0_TEI ((IRQn_Type) 4) /* SPI0 TEI (Transmission complete event) */
#define SPI0_TEI_IRQn          ((IRQn_Type) 4) /* SPI0 TEI (Transmission complete event) */
#define VECTOR_NUMBER_SPI0_ERI ((IRQn_Type) 5) /* SPI0 ERI (Error) */
#define SPI0_ERI_IRQn          ((IRQn_Type) 5) /* SPI0 ERI (Error) */
#define VECTOR_NUMBER_DMAC2_INT ((IRQn_Type) 6) /* DMAC2 INT (DMAC2 transfer end) */
#define DMAC2_INT_IRQn          ((IRQn_Type) 6) /* DMAC2 INT (DMAC2 transfer end) */
#define VECTOR_NUMBER_DMAC3_INT ((IRQn_Type) 7) /* DMAC3 INT (DMAC3 transfer end) */
#define DMAC3_INT_IRQn          ((IRQn_Type) 7) /* DMAC3 INT (DMAC3 transfer end) */
#define VECTOR_NUMBER_SPI1_TEI ((IRQn_Type) 8) /* SPI1 TEI (Transmission complete event) */
#define SPI1_TEI_IRQn          ((IRQn_Type) 8) /* SPI1 TEI (Transmission complete event) */
#define VECTOR_NUMBER_SPI1_ERI ((IRQn_Type) 9) /* SPI1 ERI (Error) */
#define SPI1_ERI_IRQn          ((IRQn_Type) 9) /* SPI1 ERI (Error) */
#define VECTOR_NUMBER_DMAC0_INT ((IRQn_Type) 10) /* DMAC0 INT (DMAC0 transfer end) */
#define DMAC0_INT_IRQn          ((IRQn_Type) 10) /* DMAC0 INT (DMAC0 transfer end) */
#define VECTOR_NUMBER_DMAC1_INT ((IRQn_Type) 11) /* DMAC1 INT (DMAC1 transfer end) */
#define DMAC1_INT_IRQn          ((IRQn_Type) 11) /* DMAC1 INT (DMAC1 transfer end) */
#define VECTOR_NUMBER_IPC_IRQ0 ((IRQn_Type) 12) /* IPC IRQ0 (CPU Mutual Interrupt 0) */
#define IPC_IRQ0_IRQn          ((IRQn_Type) 12) /* IPC IRQ0 (CPU Mutual Interrupt 0) */
#define VECTOR_NUMBER_IPC_IRQ1 ((IRQn_Type) 13) /* IPC IRQ1 (CPU Mutual Interrupt 1) */
#define IPC_IRQ1_IRQn          ((IRQn_Type) 13) /* IPC IRQ1 (CPU Mutual Interrupt 1) */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 14) /* SCI9 RXI (Receive data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 14) /* SCI9 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 15) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 15) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 16) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 16) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 17) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 17) /* SCI9 ERI (Receive error) */
#define VECTOR_NUMBER_CAN1_CHERR ((IRQn_Type) 18) /* CAN1 CHERR (Channel  error) */
#define CAN1_CHERR_IRQn          ((IRQn_Type) 18) /* CAN1 CHERR (Channel  error) */
#define VECTOR_NUMBER_CAN1_TX ((IRQn_Type) 19) /* CAN1 TX (Transmit interrupt) */
#define CAN1_TX_IRQn          ((IRQn_Type) 19) /* CAN1 TX (Transmit interrupt) */
#define VECTOR_NUMBER_CAN1_COMFRX ((IRQn_Type) 20) /* CAN1 COMFRX (Common FIFO receive interrupt) */
#define CAN1_COMFRX_IRQn          ((IRQn_Type) 20) /* CAN1 COMFRX (Common FIFO receive interrupt) */
#define VECTOR_NUMBER_CAN_GLERR ((IRQn_Type) 21) /* CAN GLERR (Global error) */
#define CAN_GLERR_IRQn          ((IRQn_Type) 21) /* CAN GLERR (Global error) */
#define VECTOR_NUMBER_CAN_RXF ((IRQn_Type) 22) /* CAN RXF (Global receive FIFO interrupt) */
#define CAN_RXF_IRQn          ((IRQn_Type) 22) /* CAN RXF (Global receive FIFO interrupt) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (23)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
