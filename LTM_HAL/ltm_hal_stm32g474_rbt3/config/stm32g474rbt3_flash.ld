/* STM32G474RBT3：Flash 128KB / SRAM 96KB（SRAM1 80KB + SRAM2 16KB）/ CCM SRAM 32KB
 *
 * CCM 优化（电流环确定性）：
 *   - 栈进 CCM SRAM（CPU 独占）：出入栈不经过 AHB，不与 DMA 争总线
 *     （SPI/ADC DMA 正在电流环执行期间搬运，栈放 CCM 后压栈/出栈零竞争）
 *   - .ccm_noinit 段放电流环路径上 CPU 独占的热变量（NOLOAD，由驱动 init 显式清零）
 *   - DMA 直接读写的缓冲（rx_buf/tx_buf/ADC DMA 缓冲）严禁放 CCM，必须留在 SRAM
 *
 * CCM 拆成两个区域（热数据向下、栈锚定顶端），区域本身即约束，避免段内跳变
 * 被 ld 计入段大小（否则 size 会把中间空隙算成 bss）
 */
MEMORY
{
    RAM      (xrw)  : ORIGIN = 0x20000000,   LENGTH = 96K    /* SRAM1 80K + SRAM2 16K */
    CCMRAM   (xrw)  : ORIGIN = 0x10000000,   LENGTH = 24K    /* CCM 热数据（仅 CPU 可访问）*/
    CCMSTACK (xrw)  : ORIGIN = 0x10006000,   LENGTH = 8K     /* CCM 栈区（顶端 8K）*/
    FLASH    (rx)   : ORIGIN = 0x8000000,    LENGTH = 128K
}

/* Entry Point */
ENTRY(Reset_Handler)

/* Sections */
SECTIONS
{
    /* 中断向量表 */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* 代码段 */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* 只读数据段 */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM异常处理段 */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    /* ARM异常索引表 */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } >FLASH

    /* 预初始化数组 */
    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH

    /* 初始化数组 */
    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH

    /* 结束数组 */
    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* 用于数据段复制到RAM的变量 */
    _sidata = LOADADDR(.data);

    /* 初始化数据段 */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)

        . = ALIGN(4);
        _edata = .;
    } >RAM AT>FLASH

    /* 未初始化数据段 (BSS) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* CCM 热数据段（NOLOAD）：电流环路径上、CPU 独占的变量
     * 使用 __attribute__((section(".ccm_noinit"))) 放入；
     * 由对应驱动 init 显式清零/初始化（如 adc_init 中 memset adc_sample）；
     * DMA 直接读写的缓冲严禁放此段 */
    .ccm_noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.ccm_noinit)
        *(.ccm_noinit.*)
        . = ALIGN(4);
    } >CCMRAM

    /* 堆：只用符号界定范围，不建输出段
     *   - 不建段 → size 不会把 96KB 可用堆算进 bss，ELF 也不会多出无内容段
     *   - 堆自 _ebss 起向上；栈在 CCM 栈区，两者物理隔离，永不相遇 */
    __HeapBase = _ebss;
    end        = __HeapBase;
    __end__    = __HeapBase;
    __HeapLimit = ORIGIN(RAM) + LENGTH(RAM) - 4;   /* 顶部 4 字节留给 BootLoader 跳转标记 */
    __BL_JUMP_FLAG = ORIGIN(RAM) + LENGTH(RAM) - 4;/* 对应 ltm_hal.c 的 BL_JUMP_FLAG_ADDR（0x20017FFC）*/

    /* 栈段：CCM 栈区（CPU 独占，出入栈不争 AHB 总线，电流环出入栈开销确定化）
     * 区域长度与 STACK_SIZE 一致，故 __StackTop 恒落在 CCM 顶端 0x10008000；
     * 热数据区（24K）与栈区（8K）由 MEMORY 区域硬隔离，不存在重叠可能 */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        __StackLimit = .;
        . = . + STACK_SIZE;
        . = ALIGN(8);
        __StackTop = .;
    } >CCMSTACK

    /* 栈顶指针，用于启动文件 */
    PROVIDE(__stack = __StackTop);

    /* CCM 两个区域不得重叠 */
    ASSERT((ORIGIN(CCMRAM) + LENGTH(CCMRAM)) <= ORIGIN(CCMSTACK),
           "Error: CCM hot-data region overlaps stack region")

    /* 栈区长度必须与 STACK_SIZE 一致（保证 __StackTop 锚定 CCM 顶端）*/
    ASSERT(SIZEOF(.stack) == LENGTH(CCMSTACK),
           "Error: STACK_SIZE mismatches CCMSTACK region length")

    /* Flash 容量检查：覆盖代码 + 只读 + 初始化数据的加载镜像 */
    ASSERT((LOADADDR(.data) + SIZEOF(.data)) <= (ORIGIN(FLASH) + LENGTH(FLASH)),
           "Error: FLASH overflow (text + rodata + data LMA)")

    /* 丢弃无用元数据段（原来整包丢弃 libc/libm/libgcc 过重，且 nano 下库名不同）
     * 注意：.ARM.exidx/.rodata 等有用段由上面的通配符收集，切勿整体丢弃 */
    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
    }
}

/* 栈大小定义（须与 CCMSTACK 区域长度一致；默认 8KB，可经 __stack_size__ 覆盖）*/
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x2000;