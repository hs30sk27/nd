/*
 * ui_config.c
 *
 * 명령(UI)에서 변경되는 설정 값들을 RAM에 보관.
 * (추후 Flash/Backup Register 저장이 필요하면 여기에서 확장)
 */

#include "ui_types.h"
#include <string.h>

static UI_Config_t s_cfg;

static void prv_set_setting_ascii(uint8_t value, char unit)
{
    /* value: 0..99 */
    s_cfg.setting_ascii[0] = (uint8_t)('0' + ((value / 10u) % 10u));
    s_cfg.setting_ascii[1] = (uint8_t)('0' + (value % 10u));
    s_cfg.setting_ascii[2] = (uint8_t)unit;
}

static void prv_init_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));

    /* 기본 NETID: ASCII '0' 10개 */
    for (uint32_t i = 0; i < UI_NET_ID_LEN; i++)
    {
        s_cfg.net_id[i] = (uint8_t)'0';
    }

    s_cfg.gw_num    = 0;
    s_cfg.max_nodes = UI_MAX_NODES;
    s_cfg.node_num  = 0;

    s_cfg.setting_value = 0;
    s_cfg.setting_unit  = 'H';
    prv_set_setting_ascii(s_cfg.setting_value, s_cfg.setting_unit);

    /* TCPIP default: 0.0.0.0:0 */
    s_cfg.tcpip_ip[0] = 0;
    s_cfg.tcpip_ip[1] = 0;
    s_cfg.tcpip_ip[2] = 0;
    s_cfg.tcpip_ip[3] = 0;
    s_cfg.tcpip_port  = 0;
}

const UI_Config_t* UI_GetConfig(void)
{
    static uint8_t inited = 0;
    if (inited == 0)
    {
        prv_init_defaults();
        inited = 1;
    }
    return &s_cfg;
}

void UI_SetNetId(const uint8_t net_id_10[UI_NET_ID_LEN])
{
    memcpy(s_cfg.net_id, net_id_10, UI_NET_ID_LEN);
}

void UI_SetGwNum(uint8_t gw_num)
{
    if (gw_num > 2u) { gw_num = 2u; }
    s_cfg.gw_num = gw_num;
}

void UI_SetMaxNodes(uint8_t max_nodes)
{
    if (max_nodes < 1u) { max_nodes = 1u; }
    if (max_nodes > UI_MAX_NODES) { max_nodes = UI_MAX_NODES; }
    s_cfg.max_nodes = max_nodes;
}

void UI_SetNodeNum(uint8_t node_num)
{
    if (node_num >= UI_MAX_NODES) { node_num = (UI_MAX_NODES - 1u); }
    s_cfg.node_num = node_num;
}

void UI_SetSetting(uint8_t value, char unit)
{
    if (value > 99u) { value = 99u; }
    if ((unit != 'M') && (unit != 'H')) { unit = 'H'; }

    s_cfg.setting_value = value;
    s_cfg.setting_unit  = unit;
    prv_set_setting_ascii(value, unit);
}

void UI_SetTcpIp(const uint8_t ip[4], uint16_t port)
{
    memcpy(s_cfg.tcpip_ip, ip, 4u);
    s_cfg.tcpip_port = port;
}
