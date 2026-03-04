#include "nd_cfg.h"
#include "main.h"
#include "stm32wlxx_hal.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#define ND_CFG_FLASH_ADDR            (0x0803F800u)
#define ND_CFG_FLASH_PAGE_SIZE       (2048u)
#define ND_CFG_MAGIC                 (0x4E444346u) /* NDCF */
#define ND_CFG_VERSION               (1u)
#define ND_CFG_CMD_MAX               (64u)

typedef struct __attribute__((packed))
{
    uint32_t    magic;
    uint16_t    version;
    uint16_t    size;
    ND_Config_t cfg;
    uint32_t    crc32;
} ND_ConfigFlash_t;

volatile ND_Config_t g_nd_cfg_dbg;

static ND_Config_t s_cfg;
static uint8_t s_inited = 0u;
static char s_rx_line[ND_CFG_CMD_MAX];
static uint8_t s_rx_len = 0u;
static char s_cmd_line[ND_CFG_CMD_MAX];
static volatile uint8_t s_cmd_ready = 0u;

static uint32_t prv_crc32(const uint8_t* data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint32_t j;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (j = 0u; j < 8u; j++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1u) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1u;
            }
        }
    }

    return ~crc;
}

static void prv_sync_dbg(void)
{
    g_nd_cfg_dbg = s_cfg;
}

static void prv_init_defaults(void)
{
    uint32_t i;
    memset(&s_cfg, 0, sizeof(s_cfg));
    for (i = 0u; i < ND_CFG_NET_ID_LEN; i++)
    {
        s_cfg.net_id[i] = (uint8_t)'0';
    }
    s_cfg.nd_num = 1u;
    s_cfg.sensor_mask = (ND_CFG_SENSOR_BIT_X |
                         ND_CFG_SENSOR_BIT_Y |
                         ND_CFG_SENSOR_BIT_Z |
                         ND_CFG_SENSOR_BIT_ADC |
                         ND_CFG_SENSOR_BIT_PULSE);
    prv_sync_dbg();
}

static void prv_apply_limits(void)
{
    if (s_cfg.nd_num < 1u)
    {
        s_cfg.nd_num = 1u;
    }
    if (s_cfg.nd_num > 50u)
    {
        s_cfg.nd_num = 50u;
    }
    s_cfg.sensor_mask &= (ND_CFG_SENSOR_BIT_X |
                          ND_CFG_SENSOR_BIT_Y |
                          ND_CFG_SENSOR_BIT_Z |
                          ND_CFG_SENSOR_BIT_ADC |
                          ND_CFG_SENSOR_BIT_PULSE);
    prv_sync_dbg();
}

static bool prv_flash_read(ND_ConfigFlash_t* out)
{
    const ND_ConfigFlash_t* p = (const ND_ConfigFlash_t*)ND_CFG_FLASH_ADDR;
    uint32_t crc;

    if (out == NULL)
    {
        return false;
    }

    memcpy(out, p, sizeof(*out));
    if ((out->magic != ND_CFG_MAGIC) ||
        (out->version != ND_CFG_VERSION) ||
        (out->size != sizeof(ND_Config_t)))
    {
        return false;
    }

    crc = prv_crc32((const uint8_t*)&out->cfg, sizeof(out->cfg));
    return (crc == out->crc32);
}

static bool prv_flash_write(void)
{
    ND_ConfigFlash_t img;
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0u;
    uint32_t offset;
    HAL_StatusTypeDef st;

    memset(&img, 0xFF, sizeof(img));
    img.magic = ND_CFG_MAGIC;
    img.version = ND_CFG_VERSION;
    img.size = sizeof(ND_Config_t);
    img.cfg = s_cfg;
    img.crc32 = prv_crc32((const uint8_t*)&img.cfg, sizeof(img.cfg));

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_BANK_1)
    erase.Banks = FLASH_BANK_1;
#endif
    erase.Page = (ND_CFG_FLASH_ADDR - FLASH_BASE) / ND_CFG_FLASH_PAGE_SIZE;
    erase.NbPages = 1u;

    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return false;
    }

    for (offset = 0u; offset < sizeof(img); offset += 8u)
    {
        uint64_t dw = 0xFFFFFFFFFFFFFFFFull;
        uint32_t copy_len = ((sizeof(img) - offset) >= 8u) ? 8u : (sizeof(img) - offset);
        memcpy(&dw, ((const uint8_t*)&img) + offset, copy_len);
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ND_CFG_FLASH_ADDR + offset, dw);
        if (st != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    (void)HAL_FLASH_Lock();
    return true;
}

static void prv_ensure_init(void)
{
    ND_ConfigFlash_t img;

    if (s_inited != 0u)
    {
        return;
    }

    if (prv_flash_read(&img))
    {
        s_cfg = img.cfg;
        prv_apply_limits();
    }
    else
    {
        prv_init_defaults();
        prv_apply_limits();
        (void)prv_flash_write();
    }

    s_inited = 1u;
}

static void prv_copy_upper_compact(char* out, size_t out_sz, const char* in)
{
    size_t n = 0u;
    while ((*in != '\0') && ((n + 1u) < out_sz))
    {
        unsigned char ch = (unsigned char)(*in++);
        if ((ch == ' ') || (ch == '\t'))
        {
            continue;
        }
        out[n++] = (char)toupper(ch);
    }
    out[n] = '\0';
}

static bool prv_starts_with(const char* s, const char* prefix)
{
    return (strncmp(s, prefix, strlen(prefix)) == 0);
}

static bool prv_parse_u8(const char* s, uint8_t* out)
{
    long v;
    char* endp;
    if ((s == NULL) || (out == NULL) || (*s == '\0'))
    {
        return false;
    }
    v = strtol(s, &endp, 10);
    if ((*endp != '\0') || (v < 0) || (v > 255))
    {
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

static bool prv_apply_sensor_cmd(const char* compact)
{
    static const struct { const char* key; uint8_t bit; } map[] = {
        { "X",   ND_CFG_SENSOR_BIT_X },
        { "Y",   ND_CFG_SENSOR_BIT_Y },
        { "Z",   ND_CFG_SENSOR_BIT_Z },
        { "ADC", ND_CFG_SENSOR_BIT_ADC },
        { "P",   ND_CFG_SENSOR_BIT_PULSE },
    };
    uint32_t i;

    for (i = 0u; i < (sizeof(map) / sizeof(map[0])); i++)
    {
        char en_key[12];
        char dis_key[12];
        (void)snprintf(en_key, sizeof(en_key), "%sEN", map[i].key);
        (void)snprintf(dis_key, sizeof(dis_key), "%sDIS", map[i].key);
        if (prv_starts_with(compact, en_key))
        {
            s_cfg.sensor_mask |= map[i].bit;
            return true;
        }
        if (prv_starts_with(compact, dis_key))
        {
            s_cfg.sensor_mask &= (uint8_t)~map[i].bit;
            return true;
        }
    }

    return false;
}

static void prv_handle_command(const char* line)
{
    char compact[ND_CFG_CMD_MAX];
    uint8_t v = 0u;
    size_t n;

    prv_copy_upper_compact(compact, sizeof(compact), line);
    n = strlen(compact);
    while ((n > 0u) && (compact[n - 1u] == ':'))
    {
        compact[--n] = '\0';
    }

    if (prv_starts_with(compact, "NETID:"))
    {
        const char* val = compact + 6;
        uint32_t i;
        for (i = 0u; i < ND_CFG_NET_ID_LEN; i++)
        {
            s_cfg.net_id[i] = (val[i] != '\0') ? (uint8_t)val[i] : (uint8_t)'0';
        }
        prv_apply_limits();
        (void)prv_flash_write();
        return;
    }

    if (prv_starts_with(compact, "NDNUM:"))
    {
        if (prv_parse_u8(compact + 6, &v))
        {
            s_cfg.nd_num = v;
            prv_apply_limits();
            (void)prv_flash_write();
        }
        return;
    }

    if (prv_apply_sensor_cmd(compact))
    {
        prv_apply_limits();
        (void)prv_flash_write();
        return;
    }
}

void ND_Config_Init(void)
{
    prv_ensure_init();
}

void ND_Config_Process(void)
{
    char line[ND_CFG_CMD_MAX];

    prv_ensure_init();

    if (s_cmd_ready == 0u)
    {
        return;
    }

    __disable_irq();
    memcpy(line, s_cmd_line, sizeof(line));
    s_cmd_line[0] = '\0';
    s_cmd_ready = 0u;
    __enable_irq();

    prv_handle_command(line);
}

void ND_Cmd_OnRxByte(uint8_t ch)
{
    if ((ch == '\r') || (ch == '\n'))
    {
        if ((s_rx_len != 0u) && (s_cmd_ready == 0u))
        {
            s_rx_line[s_rx_len] = '\0';
            memcpy(s_cmd_line, s_rx_line, sizeof(s_cmd_line));
            s_cmd_ready = 1u;
        }
        s_rx_len = 0u;
        return;
    }

    if ((ch < 0x20u) || (ch > 0x7Eu))
    {
        return;
    }

    if (s_rx_len < (ND_CFG_CMD_MAX - 1u))
    {
        s_rx_line[s_rx_len++] = (char)ch;
    }
    else
    {
        s_rx_len = 0u;
    }
}

const ND_Config_t* ND_Config_Get(void)
{
    prv_ensure_init();
    return &s_cfg;
}

uint8_t ND_Config_GetSensorMask(void)
{
    prv_ensure_init();
    return s_cfg.sensor_mask;
}
