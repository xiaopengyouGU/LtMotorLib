/* lt_cogging.c */
#include "analysis/ident/lt_ident.h"
#include "math/basic/lt_math.h"
#include <string.h>

#define MAX_TABLE_SIZE 512

typedef struct {
    uint16_t table_size;
    float   table[MAX_TABLE_SIZE];
    uint8_t filled[MAX_TABLE_SIZE]; /* 标记每个位置是否已采 */
    uint16_t count;
    uint8_t  done;                  /* 采样完毕标志,0：未采完，1：已采完 */
} lt_cogging_obj;

static lt_cogging_obj cogging_obj;
static lt_cogging_obj * cog = &cogging_obj;

void lt_cogging_init(uint16_t table_size)
{
    if(table_size > MAX_TABLE_SIZE) table_size = MAX_TABLE_SIZE;
    memset(cog, 0, sizeof(lt_cogging_obj));
    cog->table_size = table_size;
}

void lt_cogging_start(void)
{
    cog->done = 0;
    cog->count = 0;
}

void lt_cogging_add(float pos_deg, float iq)
{
    if (cog->done) return;
    if (cog->table_size == 0) return;

    pos_deg = lt_normalize_quick(pos_deg, 360.0f);
    uint8_t table_size = cog->table_size;
    uint16_t idx = (uint16_t)(pos_deg / 360.0f * table_size);
    if (idx >= table_size) idx = table_size - 1;

    /* 已采过 → 丢弃 */
    if (cog->filled[idx]) return;

    cog->table[idx] = iq;
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

#define INV_360_DEG     0.0027778f      /* 1.0f / 360.0f */      

float lt_cogging_get(float pos_deg)
{
    if (!cog->done) return 0.0f;
    if (cog->table_size == 0) return 0.0f;
    uint16_t table_size = cog->table_size;

    pos_deg = lt_normalize_quick(pos_deg, 360.0f);  /* 快速归一化到 [0, 360°）*/

    float idx_f = pos_deg * table_size * INV_360_DEG;
    uint16_t i0 = (uint16_t)idx_f;
    float frac = idx_f - (float)i0;

    if (i0 >= table_size) i0 = table_size - 1;
    uint16_t i1 = i0 + 1;
    if (i1 >= table_size) i1 = 0;

    return cog->table[i0] + (cog->table[i1] - cog->table[i0]) * frac;
}