/*
                  Linker File for Renesas FSP
*/

INCLUDE memory_regions.ld
INCLUDE fsp_gen.ld

/* 堆起点：_sbrk 依赖该符号 */
PROVIDE (end = .);
