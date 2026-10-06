/* lt_cogging.c */
#include "analysis/ident/lt_ident.h"
#include <string.h>

#define MAX_TABLE_SIZE 512

typedef struct {
    uint16_t table_size;
    uint32_t ratio_q16;                 /* table_size/reso（Q16）：count → 表索引 */
    int32_t  table[MAX_TABLE_SIZE];     /* 等效 Q 轴电流（Q15 标幺）*/
    uint8_t  filled[MAX_TABLE_SIZE];    /* 标记每个位置是否已采 */
    uint16_t count;
    uint8_t  done;                      /* 采样完毕标志,0：未采完，1：已采完 */
} lt_cogging_obj;

static lt_cogging_obj cogging_obj;
static lt_cogging_obj * cog = &cogging_obj;

void lt_cogging_init(uint16_t table_size, uint32_t reso)
{
    if(table_size > MAX_TABLE_SIZE) table_size = MAX_TABLE_SIZE;
    memset(cog, 0, sizeof(lt_cogging_obj));
    cog->table_size = table_size;
    /* 索引 = count·table_size/reso，折成 Q16 常数乘；count < reso，乘积落在 32 位内 */
    cog->ratio_q16  = reso ? ((((uint32_t)table_size << 16) + reso / 2) / reso) : 0;
}

void lt_cogging_start(void)
{
    cog->done = 0;
    cog->count = 0;
}

/* pos_count：单圈计数 0~reso-1；Iq：等效 Q 轴电流（Q15 标幺）*/
void lt_cogging_add(uint32_t pos_count, int32_t Iq)
{
    uint16_t table_size = cog->table_size;
    if (cog->done)      return;
    if (!table_size)    return;

    uint16_t idx = (uint16_t)(((uint32_t)pos_count * cog->ratio_q16) >> 16);
    if (idx >= table_size)  idx = table_size - 1;
    /* 已采过 → 丢弃 */
    if (cog->filled[idx]) return;
    cog->table[idx] = Iq;
    cog->filled[idx] = 1;
    cog->count++;
    if (cog->count >= table_size) {
        cog->done = 1;
    }
}

uint8_t lt_cogging_is_done(void)
{
    return cog->done;
}

/* 单圈计数 → 等效 Q 轴电流（Q15 标幺），表内线性插值 */
int32_t lt_cogging_get(uint32_t pos_count)
{
    uint16_t table_size = cog->table_size;
    if (!cog->done)     return 0;
    if (!table_size)    return 0;

    uint32_t idx_q16 = (uint32_t)pos_count * cog->ratio_q16;
    uint16_t i0      = (uint16_t)(idx_q16 >> 16);
    uint32_t frac    = idx_q16 & 0xFFFF;       /* 索引小数部分（Q16），插值权重 */

    if (i0 >= table_size) i0 = table_size - 1;
    uint16_t i1 = i0 + 1;
    if (i1 >= table_size) i1 = 0;
    int32_t t0 = cog->table[i0];
    int32_t t1 = cog->table[i1];

    return t0 + (int32_t)(((int64_t)(t1 - t0) * frac) >> 16);
}
