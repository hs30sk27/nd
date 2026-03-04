/*
 * ui_types.h
 *
 * Core/ui 모듈에서 공통으로 사용하는 타입/구조체 정의
 */

#ifndef UI_TYPES_H
#define UI_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "ui_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* 공통 설정(런타임)                                                          */
/* -------------------------------------------------------------------------- */
typedef struct
{
    /* 공통: 네트워크 아이디(10바이트 고정) */
    uint8_t  net_id[UI_NET_ID_LEN];

    /* Gateway 전용 */
    uint8_t  gw_num;        /* 0..2 */
    uint8_t  max_nodes;     /* 1..50 (GW only: ND NUM:xx) */

    /* Node 전용 */
    uint8_t  node_num;      /* 0..49 (ND NUM:xx) */

    /* 테스트/동작 설정 (GW SETTING 명령 값)
     *  - setting_value: 0..99
     *  - setting_unit : 'M' 또는 'H'
     *  - setting_ascii: "01M" 형태 3바이트(널 종료 없음)
     */
    uint8_t  setting_value;
    char     setting_unit;
    uint8_t  setting_ascii[3];

    /* Gateway CATM1 서버 설정 (TCPIP 명령) */
    uint8_t  tcpip_ip[4];   /* xxx.xxx.xxx.xxx */
    uint16_t tcpip_port;    /* 0..65535 */

} UI_Config_t;

const UI_Config_t* UI_GetConfig(void);

/* 설정 변경 API (명령 파서에서 사용) */
void UI_SetNetId(const uint8_t net_id_10[UI_NET_ID_LEN]);
void UI_SetGwNum(uint8_t gw_num);
void UI_SetMaxNodes(uint8_t max_nodes);
void UI_SetNodeNum(uint8_t node_num);
void UI_SetSetting(uint8_t value, char unit);
void UI_SetTcpIp(const uint8_t ip[4], uint16_t port);

/* -------------------------------------------------------------------------- */
/* GPIO 이벤트 플래그 (ISR -> main deferred)                                   */
/* -------------------------------------------------------------------------- */
typedef enum
{
    UI_GPIO_EVT_NONE      = 0,
    UI_GPIO_EVT_TEST_KEY  = (1u << 0),
    UI_GPIO_EVT_OP_KEY    = (1u << 1),
    UI_GPIO_EVT_PULSE_IN  = (1u << 2),
} UI_GpioEventBits_t;

/* -------------------------------------------------------------------------- */
/* 내부 명령 파서 결과                                                        */
/* -------------------------------------------------------------------------- */
typedef enum
{
    UI_CMD_OK = 0,
    UI_CMD_ERROR,
} UI_CmdStatus_t;

#ifdef __cplusplus
}
#endif

#endif /* UI_TYPES_H */
