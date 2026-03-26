#include "ui_cmd.h"
#include "ui_types.h"
#include "ui_uart.h"
#include "ui_time.h"
#include "ui_ble.h"

#include "stm32wlxx_hal.h" /* __weak */
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* -------------------------------------------------------------------------- */
/* Hook functions (GW/ND에서 필요 시 override)                                 */
/* -------------------------------------------------------------------------- */
__weak void UI_Hook_OnConfigChanged(void) {}
__weak void UI_Hook_OnTimeChanged(void) {}
__weak void UI_Hook_OnBeaconOnceRequested(void) {}
__weak void UI_Hook_OnBleEndRequested(void) {}
__weak bool UI_Hook_OnTestStartRequested(void) { return false; }
__weak bool UI_Hook_OnSyncRequested(void) { return false; }

/* -------------------------------------------------------------------------- */
static void prv_send_ok(void)
{
    UI_UART_SendString("OK\r\n");
}

static void prv_send_error(void)
{
    UI_UART_SendString("ERROR\r\n");
}

static void prv_send_fail(void)
{
    UI_UART_SendString("FAIL\r\n");
}

static const char* prv_skip_spaces(const char* s)
{
    while (s && *s && isspace((unsigned char)*s)) { s++; }
    return s;
}

/* CR/LF 제거용 (line은 이미 null-terminated 가정) */
static void prv_rstrip(char* s)
{
    size_t n = strlen(s);
    while (n > 0u)
    {
        char c = s[n-1u];
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t')
        {
            s[n-1u] = '\0';
            n--;
        }
        else
        {
            break;
        }
    }
}

static int prv_parse_u8_dec(const char* s, uint8_t* out)
{
    /* 0..255 */
    unsigned v = 0;
    int cnt = 0;
    while (*s && isdigit((unsigned char)*s) && cnt < 3)
    {
        v = v*10u + (unsigned)(*s - '0');
        s++;
        cnt++;
    }
    if (cnt == 0) return 0;
    if (v > 255u) return 0;
    *out = (uint8_t)v;
    return cnt;
}

static bool prv_cmd_equals_relaxed(const char* s, const char* base)
{
    if ((s == NULL) || (base == NULL))
    {
        return false;
    }

    while (*base != '\0')
    {
        if (toupper((unsigned char)*s) != toupper((unsigned char)*base))
        {
            return false;
        }
        s++;
        base++;
    }

    while ((*s == ' ') || (*s == '\t'))
    {
        s++;
    }

    if (*s == ':')
    {
        s++;
        while (isspace((unsigned char)*s))
        {
            s++;
        }
    }

    return (*s == '\0');
}


static size_t prv_netid_copy_padded(uint8_t out[UI_NET_ID_LEN], const char* id)
{
    size_t len = (id != NULL) ? strlen(id) : 0u;

    if ((len == 0u) || (len > UI_NET_ID_LEN))
    {
        return 0u;
    }

    memset(out, 0, UI_NET_ID_LEN);
    memcpy(out, id, len);
    return len;
}

static bool prv_commit_config_changed(void)
{
    if (!UI_Config_Save())
    {
        prv_send_error();
        return false;
    }

    prv_send_ok();
    UI_Hook_OnConfigChanged();
    return true;
}
static bool prv_commit_sensor_mask(uint8_t sensor_mask)
{
    UI_SetSensorEnableMask(sensor_mask);
    return prv_commit_config_changed();
}


static void prv_send_setting_read(void)
{
    const UI_Config_t* cfg = UI_GetConfig();
    char netid[UI_NET_ID_LEN + 1u];
    char line[192];

    memcpy(netid, cfg->net_id, UI_NET_ID_LEN);
    netid[UI_NET_ID_LEN] = '\0';

    (void)snprintf(line, sizeof(line), "NETID:%s\r\n", netid);
    UI_UART_SendString(line);

    (void)snprintf(line, sizeof(line), "ND NUM:%u\r\n", cfg->node_num);
    UI_UART_SendString(line);

    (void)snprintf(line, sizeof(line), "ICM:%s\r\n",
                   (cfg->sensor_en_mask & UI_SENSOR_EN_ICM20948) ? "EN" : "DIS");
    UI_UART_SendString(line);

    (void)snprintf(line, sizeof(line), "ADC:%s\r\n",
                   (cfg->sensor_en_mask & UI_SENSOR_EN_ADC) ? "EN" : "DIS");
    UI_UART_SendString(line);

    (void)snprintf(line, sizeof(line), "PULSE:%s\r\n",
                   (cfg->sensor_en_mask & UI_SENSOR_EN_PULSE) ? "EN" : "DIS");
    UI_UART_SendString(line);

    (void)snprintf(line, sizeof(line), "SETTING:%c%c%c\r\n",
                   (char)cfg->setting_ascii[0],
                   (char)cfg->setting_ascii[1],
                   (char)cfg->setting_ascii[2]);
    UI_UART_SendString(line);
}

static void prv_apply_nd_ble_name(uint8_t node_num)
{
    char ble_name[16];
    (void)snprintf(ble_name, sizeof(ble_name), "BT ND %u", (unsigned)node_num);
    (void)UI_BLE_ApplyDeviceName(ble_name);
}

void UI_Cmd_ProcessLine(const char* line_in)
{
    if (line_in == NULL) { return; }

    /*
     * BLE 세션 유지/연장은 물리 버튼으로만 허용한다.
     * 명령 수신만으로 timeout을 재연장하지 않는다.
     */

    /* line_in은 상위에서 buffer를 넘겨주므로 안전하게 로컬 복사 */
    char line[UI_UART_LINE_MAX];
    (void)snprintf(line, sizeof(line), "%s", line_in);
    prv_rstrip(line);

    /*
     * 최종 요구사항:
     *   - UART1 명령은 반드시 "<CMD>CRLF" 형태로만 동작
     *   - 블루투스 시작 시 튀는 데이터/쓰레기 데이터에는 아무 응답도 하지 않음
     *
     * 따라서 '<'로 시작하지 않으면 그냥 무시(return).
     */
    const char* s0 = prv_skip_spaces(line);
    if (s0 == NULL || *s0 != '<')
    {
        return;
    }

    /* 끝이 '>'가 아니면 미완성/쓰레기 -> 무시 */
    size_t n0 = strlen(s0);
    if (n0 < 3u || s0[n0 - 1u] != '>')
    {
        return;
    }

    /* 프레임 '<', '>' 제거 */
    char* s = (char*)prv_skip_spaces(line);
    if (*s == '<')
    {
        s++;
    }

    s = (char*)prv_skip_spaces(s);
    prv_rstrip(s);
    size_t n = strlen(s);
    if (n > 0u && s[n-1u] == '>')
    {
        s[n-1u] = '\0';
        prv_rstrip(s);
    }

    const char* p = s;
    if (*p == '\0') { return; }

    /* -------------------- SETTING READ ------------------ */
    if (prv_cmd_equals_relaxed(p, "SETTING READ"))
    {
        prv_send_setting_read();
        prv_send_ok();
        return;
    }

    /* -------------------- TIME CHECK -------------------- */
    if ((strcmp(p, "TIME CHECK") == 0) || (strcmp(p, "TIME CHECK:") == 0))
    {
        char ts[48];
        UI_Time_FormatNow(ts, sizeof(ts));
        UI_UART_SendString(ts);
        UI_UART_SendString("\r\n");
        prv_send_ok();
        return;
    }

    /* -------------------- TIME:... ---------------------- */
    if (strncmp(p, "TIME:", 5) == 0)
    {
        if (UI_Time_SetFromString(p))
        {
            prv_send_ok();
            UI_Hook_OnTimeChanged();
        }
        else
        {
            prv_send_error();
        }
        return;
    }

    /* -------------------- NETID:UTF8/ASCII -------------- */
    if (strncmp(p, "NETID:", 6) == 0)
    {
        const char* id = p + 6;
        uint8_t net_id[UI_NET_ID_LEN];

        if (prv_netid_copy_padded(net_id, id) == 0u)
        {
            prv_send_error();
            return;
        }

        UI_SetNetId(net_id);
        (void)prv_commit_config_changed();
        return;
    }

    /* -------------------- ND NUM:xx ---------------------- */
    if (strncmp(p, "ND NUM:", 7) == 0)
    {
        uint8_t v = 0;
        if (prv_parse_u8_dec(p + 7, &v) <= 0)
        {
            prv_send_error();
            return;
        }

        /* ND: ND NUM:xx = 자기 노드 번호 설정 (0..49) */
        if (v < UI_MAX_NODES)
        {
            UI_SetNodeNum(v);
            if (prv_commit_config_changed())
            {
                prv_apply_nd_ble_name(v);
            }
        }
        else
        {
            prv_send_error();
        }
        return;
    }

    /* -------------------- ICM EN/DIS -------------------- */
    if (prv_cmd_equals_relaxed(p, "ICM EN"))
    {
        const UI_Config_t* cfg = UI_GetConfig();
        (void)prv_commit_sensor_mask((uint8_t)(cfg->sensor_en_mask | UI_SENSOR_EN_ICM20948));
        return;
    }
    if (prv_cmd_equals_relaxed(p, "ICM DIS"))
    {
        const UI_Config_t* cfg = UI_GetConfig();
        (void)prv_commit_sensor_mask((uint8_t)(cfg->sensor_en_mask & (uint8_t)~UI_SENSOR_EN_ICM20948));
        return;
    }

    /* -------------------- ADC EN/DIS -------------------- */
    if (prv_cmd_equals_relaxed(p, "ADC EN"))
    {
        const UI_Config_t* cfg = UI_GetConfig();
        (void)prv_commit_sensor_mask((uint8_t)(cfg->sensor_en_mask | UI_SENSOR_EN_ADC));
        return;
    }
    if (prv_cmd_equals_relaxed(p, "ADC DIS"))
    {
        const UI_Config_t* cfg = UI_GetConfig();
        (void)prv_commit_sensor_mask((uint8_t)(cfg->sensor_en_mask & (uint8_t)~UI_SENSOR_EN_ADC));
        return;
    }

    /* -------------------- PULSE EN/DIS ------------------ */
    if (prv_cmd_equals_relaxed(p, "PULSE EN"))
    {
        const UI_Config_t* cfg = UI_GetConfig();
        (void)prv_commit_sensor_mask((uint8_t)(cfg->sensor_en_mask | UI_SENSOR_EN_PULSE));
        return;
    }
    if (prv_cmd_equals_relaxed(p, "PULSE DIS"))
    {
        const UI_Config_t* cfg = UI_GetConfig();
        (void)prv_commit_sensor_mask((uint8_t)(cfg->sensor_en_mask & (uint8_t)~UI_SENSOR_EN_PULSE));
        return;
    }

    /* -------------------- SETTING:xxM/H ------------------ */
    if (strncmp(p, "SETTING:", 8) == 0)
    {
        const char* q = p + 8;
        uint8_t v = 0;
        int n = prv_parse_u8_dec(q, &v);
        if (n <= 0)
        {
            prv_send_error();
            return;
        }
        q += n;
        char unit = *q;
        if ((unit != 'M') && (unit != 'H'))
        {
            prv_send_error();
            return;
        }

        /* ND 로컬 테스트/정상 주기 설정 */
        UI_SetSetting(v, unit);
        (void)prv_commit_config_changed();
        return;
    }

    /* -------------------- SYNC --------------------------- */
    if (prv_cmd_equals_relaxed(p, "SYNC"))
    {
        if (!UI_Hook_OnSyncRequested())
        {
            prv_send_fail();
        }
        return;
    }

    /* -------------------- TEST START --------------------- */
    if (prv_cmd_equals_relaxed(p, "TEST START"))
    {
        if (UI_Hook_OnTestStartRequested())
        {
            prv_send_ok();
        }
        else
        {
            prv_send_error();
        }
        return;
    }

    /* -------------------- BLE END ------------------------ */
    if ((strncmp(p, "BLE END", 7) == 0) || (strncmp(p, "BLE END:", 8) == 0))
    {
        prv_send_ok();
        UI_Hook_OnBleEndRequested();
        return;
    }

    /* Unknown */
    prv_send_error();
}
