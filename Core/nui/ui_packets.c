#include "ui_packets.h"
#include "ui_crc16.h"
#include <string.h>

static void prv_put_u16_le(uint8_t* p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t prv_get_u16_le(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void prv_put_u32_le(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t prv_get_u32_le(const uint8_t* p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

uint8_t UI_Pkt_BuildBeacon(uint8_t out[UI_BEACON_PAYLOAD_LEN],
                           const uint8_t net_id[UI_NET_ID_LEN],
                           const UI_DateTime_t* dt,
                           const uint8_t setting_ascii[3])
{
    /* 10 + 6 + 3 + CRC16(2) */
    uint8_t* p = out;

    memcpy(p, net_id, UI_NET_ID_LEN); p += UI_NET_ID_LEN;

    /* TIME 6 bytes: YY,MM,DD,hh,mm,ss */
    p[0] = (uint8_t)(dt->year % 100u);
    p[1] = dt->month;
    p[2] = dt->day;
    p[3] = dt->hour;
    p[4] = dt->min;
    p[5] = dt->sec;
    p += 6;

    memcpy(p, setting_ascii, 3); p += 3;

    uint16_t crc = UI_CRC16_CCITT(out, (size_t)(UI_NET_ID_LEN + 6u + 3u), UI_CRC16_INIT);
    prv_put_u16_le(p, crc);

    return UI_BEACON_PAYLOAD_LEN;
}

bool UI_Pkt_ParseBeacon(const uint8_t* buf, uint16_t len, UI_Beacon_t* out)
{
    if (len < UI_BEACON_PAYLOAD_LEN) return false;

    uint16_t crc_rx = prv_get_u16_le(&buf[UI_NET_ID_LEN + 6u + 3u]);
    uint16_t crc    = UI_CRC16_CCITT(buf, (size_t)(UI_NET_ID_LEN + 6u + 3u), UI_CRC16_INIT);
    if (crc != crc_rx) return false;

    memcpy(out->net_id, buf, UI_NET_ID_LEN);

    const uint8_t* p = &buf[UI_NET_ID_LEN];

    out->dt.year  = (uint16_t)(2000u + (uint16_t)p[0]);
    out->dt.month = p[1];
    out->dt.day   = p[2];
    out->dt.hour  = p[3];
    out->dt.min   = p[4];
    out->dt.sec   = p[5];
    out->dt.centi = 0;

    memcpy(out->setting_ascii, &buf[UI_NET_ID_LEN + 6u], 3u);

    return true;
}

uint8_t UI_Pkt_BuildNodeData(uint8_t out[UI_NODE_PAYLOAD_LEN],
                             const UI_NodeData_t* in)
{
    uint8_t* p = out;

    p[0] = in->node_num; p += 1;
    memcpy(p, in->net_id, UI_NET_ID_LEN); p += UI_NET_ID_LEN;

    p[0] = in->batt_lvl; p += 1;
    p[0] = (uint8_t)in->temp_c; p += 1;
    prv_put_u16_le(p, in->beacon_cnt); p += 2;

    prv_put_u16_le(p, (uint16_t)in->x); p += 2;
    prv_put_u16_le(p, (uint16_t)in->y); p += 2;
    prv_put_u16_le(p, (uint16_t)in->z); p += 2;

    prv_put_u16_le(p, in->adc); p += 2;
    prv_put_u32_le(p, in->pulse_cnt); p += 4;

    uint16_t crc = UI_CRC16_CCITT(out, (size_t)(UI_NODE_PAYLOAD_LEN - 2u), UI_CRC16_INIT);
    prv_put_u16_le(p, crc);

    return UI_NODE_PAYLOAD_LEN;
}

bool UI_Pkt_ParseNodeData(const uint8_t* buf, uint16_t len, UI_NodeData_t* out)
{
    if (len < UI_NODE_PAYLOAD_LEN) return false;

    uint16_t crc_rx = prv_get_u16_le(&buf[UI_NODE_PAYLOAD_LEN - 2u]);
    uint16_t crc    = UI_CRC16_CCITT(buf, (size_t)(UI_NODE_PAYLOAD_LEN - 2u), UI_CRC16_INIT);
    if (crc != crc_rx) return false;

    const uint8_t* p = buf;

    out->node_num = p[0]; p += 1;
    memcpy(out->net_id, p, UI_NET_ID_LEN); p += UI_NET_ID_LEN;

    out->batt_lvl = p[0]; p += 1;
    out->temp_c = (int8_t)p[0]; p += 1;
    out->beacon_cnt = prv_get_u16_le(p); p += 2;

    out->x = (int16_t)prv_get_u16_le(p); p += 2;
    out->y = (int16_t)prv_get_u16_le(p); p += 2;
    out->z = (int16_t)prv_get_u16_le(p); p += 2;

    out->adc = prv_get_u16_le(p); p += 2;
    out->pulse_cnt = prv_get_u32_le(p); p += 4;

    return true;
}
