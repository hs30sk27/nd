/*
 * ui_packets.h
 *
 * LoRa 패킷 포맷(비콘/노드데이터) 정의 + encode/decode
 *
 * 요구사항에서 명시된 payload 구성 그대로 구현:
 *
 * [BEACON]
 *  - NETID 10 bytes
 *  - TIME  6 bytes (YY,MM,DD,hh,mm,ss)
 *  - TEST  3 bytes ("01M" 같은 SETTING ASCII)
 *  - CRC16 2 bytes (CCITT)
 *
 * [NODE DATA]
 *  - NODE_NUM 1 byte
 *  - NETID    10 bytes
 *  - BATT     uint8  (1=normal, 0=low)
 *  - TEMP     int8   ('C) range -50..100
 *  - BCN_CNT  uint16
 *  - X,Y,Z    int16 each
 *  - ADC      uint16
 *  - PULSE    uint32
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

#define UI_BEACON_PAYLOAD_LEN   (21u)
#define UI_NODE_PAYLOAD_LEN     (29u)

/* 비콘 파싱 결과 */
typedef struct
{
    uint8_t net_id[UI_NET_ID_LEN];
    UI_DateTime_t dt;        /* centi는 사용하지 않음(전송에 포함 X) */
    uint8_t setting_ascii[3];
} UI_Beacon_t;

/* 노드 데이터 파싱/빌드용 */
typedef struct
{
    uint8_t  node_num;       /* 0..49 */
    uint8_t  net_id[UI_NET_ID_LEN];

    uint8_t  batt_lvl;       /* 1=normal, 0=low, 0xFF=internal invalid */
    int8_t   temp_c;         /* -50..100'C, UI_NODE_TEMP_INVALID_C=internal invalid */

    uint16_t beacon_cnt;

    int16_t  x;
    int16_t  y;
    int16_t  z;

    uint16_t adc;

    uint32_t pulse_cnt;

} UI_NodeData_t;

/* 빌드/파싱 */
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
