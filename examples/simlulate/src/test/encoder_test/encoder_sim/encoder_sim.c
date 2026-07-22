#include "encoder_test/encoder_sim/encoder_sim.h"
#include "math/crc/lt_crc.h"
#include "bsp/encoder/drv_spi.h"
#include <math.h>
#include <string.h>

/* ============ 内部状态 ============ */
static encoder_sim_t s_axes[Encoder_Axis_Num];  /* 0=主轴, 1=从轴 */
static int s_running = 0;

/* ============ 私有辅助 ============ */
static double norm_angle(double a) {
    a = fmod(a, 2.0 * ENCODER_SIM_PI);
    return a < 0 ? a + 2.0 * ENCODER_SIM_PI : a;
}

static uint32_t angle_to_count(double a) {
    return (uint32_t)(norm_angle(a) / (2.0 * ENCODER_SIM_PI) * ENCODER_SIM_CPR) 
           & (ENCODER_SIM_CPR - 1);
}

/* ============ 接口实现 ============ */
void encoder_sim_init(double init_spindle_turns) {
    s_axes[Spindle_Axis].position = init_spindle_turns * 2.0 * ENCODER_SIM_PI;
    s_axes[Spindle_Axis].velocity = 0.0;
    s_axes[Spindle_Axis].count = angle_to_count(s_axes[Spindle_Axis].position);
    
    s_axes[Driven_Axis].position = -init_spindle_turns * ENCODER_SIM_GEAR_RATIO * 2.0 * ENCODER_SIM_PI;
    s_axes[Driven_Axis].velocity = 0.0;
    s_axes[Driven_Axis].count = angle_to_count(s_axes[Driven_Axis].position);
    
    s_running = 0;
}

void encoder_sim_set_speed(double speed_rpm) {
    double omega = speed_rpm * 2.0 * ENCODER_SIM_PI / 60.0;
    s_axes[Spindle_Axis].velocity = omega;
    s_axes[Driven_Axis].velocity = -omega * ENCODER_SIM_GEAR_RATIO;
    s_running = 1;
}

uint32_t encoder_sim_get_count(encoder_axis_t axis) {
    return s_axes[axis].count;
}

double encoder_sim_get_turns(encoder_axis_t axis) {
    return s_axes[axis].position / (2.0 * ENCODER_SIM_PI);
}

void encoder_sim_step(void) {
    if (!s_running) return;
    
    double dt = 1.0 / ENCODER_SIM_CALL_FREQ;
    for (int i = 0; i < 2; i++) {
        s_axes[i].position += s_axes[i].velocity * dt;
        s_axes[i].count = angle_to_count(s_axes[i].position);
    }
}

float encoder_sim_get_freq(void) {
    return ENCODER_SIM_CALL_FREQ;
}

/*************************************************************************************/
/* 此处实现模拟的 SPI数据调用接口，spi_write_read，每次读取返回3个字(16-bit)，共48-bit */
static void mt6835_encode(uint32_t count, uint16_t rx[3]);

void spi_init(void)                 /* 模拟 SPI 初始化 */
{

}

int spi_write_read(spi_id_t id, uint16_t *tx_buf, uint16_t *rx_buf, uint32_t n_words) {
    static uint16_t spindle_rx[3];
    static uint16_t driven_rx[3];
    
    encoder_sim_step();  /* 每次SPI读取前更新状态 */
    
    mt6835_encode(encoder_sim_get_count(Spindle_Axis), spindle_rx);
    mt6835_encode(encoder_sim_get_count(Driven_Axis), driven_rx);
    
    if (id == SPI_1) {
        memcpy(rx_buf, spindle_rx, n_words * sizeof(uint16_t));
        return 1;
    } else if (id == SPI_0) {
        memcpy(rx_buf, driven_rx, n_words * sizeof(uint16_t));
        return 1;
    }
    return 0;
}

static void mt6835_encode(uint32_t count, uint16_t rx[3]) {
    uint32_t raw = count << 3;
    uint8_t data[4];
    
    data[0] = (raw >> 13) & 0xFF;
    data[1] = (raw >> 5) & 0xFF;
    data[2] = ((raw & 0x1F) << 3);
    data[3] = lt_crc8_check(data, 3);
    
    rx[0] = 0x0000;
    rx[1] = ((uint16_t)data[0] << 8) | data[1];
    rx[2] = ((uint16_t)data[2] << 8) | data[3];
}
