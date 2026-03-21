/*
 * ui_packets.h
 *
 * LoRa 패킷 포맷(비콘/노드데이터) 정의 + encode/decode
 *
 * [BEACON]
 *  - NETID UI_NET_ID_LEN bytes
 *  - TIME  6 bytes (YY,MM,DD,hh,mm,ss)
 *  - TEST  3 bytes ("01M" 같은 SETTING ASCII)
 *  - CRC16 2 bytes (CCITT)
 *
 * [NODE DATA]
 *  - NODE_NUM 1 byte
 *  - NETID    UI_NET_ID_LEN bytes
 *  - BATT     uint8  (1=normal, 0=low)
 *  - TEMP     int8   ('C) range -50..100
 *  - BCN_CNT  uint16
 *  - X,Y,Z    uint16 each (scaled 0..49999)
 *  - ADC      uint16
 *  - PULSE    uint32
 *  - SENSOR_EN 1 byte (bit0=ICM20948, bit1=ADC, bit2=PULSE)
 *  - CRC16    2 bytes
 */

#ifndef UI_PACKETS_H
#define UI_PACKETS_H

#include <stdint.h>
#include <stdbool.h>
#include "ui_types.h"
#include "ui_time.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UI_BEACON_PAYLOAD_LEN   (UI_NET_ID_LEN + 11u)
#define UI_NODE_PAYLOAD_LEN     (UI_NET_ID_LEN + 20u)

typedef struct
{
    uint8_t net_id[UI_NET_ID_LEN];
    UI_DateTime_t dt;
    uint8_t setting_ascii[3];
} UI_Beacon_t;

typedef struct
{
    uint8_t  node_num;
    uint8_t  net_id[UI_NET_ID_LEN];

    uint8_t  batt_lvl;
    int8_t   temp_c;

    uint16_t beacon_cnt;

    uint16_t x;
    uint16_t y;
    uint16_t z;

    uint16_t adc;

    uint32_t pulse_cnt;
    uint8_t  sensor_en_mask;

} UI_NodeData_t;

uint8_t UI_Pkt_BuildBeacon(uint8_t out[UI_BEACON_PAYLOAD_LEN],
                           const uint8_t net_id[UI_NET_ID_LEN],
                           const UI_DateTime_t* dt_no_centi,
                           const uint8_t setting_ascii[3]);

bool UI_Pkt_ParseBeacon(const uint8_t* buf, uint16_t len, UI_Beacon_t* out);

uint8_t UI_Pkt_BuildNodeData(uint8_t out[UI_NODE_PAYLOAD_LEN],
                             const UI_NodeData_t* in);

bool UI_Pkt_ParseNodeData(const uint8_t* buf, uint16_t len, UI_NodeData_t* out);

#ifdef __cplusplus
}
#endif

#endif /* UI_PACKETS_H */
