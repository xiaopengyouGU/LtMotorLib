#ifndef ENCODER_TEST_H
#define ENCODER_TEST_H

/* 绝对值编码器模块与PLL测速模块 测试用例 */
int encoder_speed_test(void);        /* PLL测速测试，验证算法逻辑与性能 */
int encoder_test(void);              /* 绝对值编码器测试，验证上电解圈与SPI数据解析 */

#endif