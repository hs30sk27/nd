#ifndef ND_CFG_H
#define ND_CFG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ND_CFG_NET_ID_LEN            (10u)
#define ND_CFG_SENSOR_BIT_X          (1u << 0)
#define ND_CFG_SENSOR_BIT_Y          (1u << 1)
#define ND_CFG_SENSOR_BIT_Z          (1u << 2)
#define ND_CFG_SENSOR_BIT_ADC        (1u << 3)
#define ND_CFG_SENSOR_BIT_PULSE      (1u << 4)

typedef struct
{
    uint8_t net_id[ND_CFG_NET_ID_LEN];
    uint8_t nd_num;
    uint8_t sensor_mask;
} ND_Config_t;

extern volatile ND_Config_t g_nd_cfg_dbg;

void ND_Config_Init(void);
void ND_Config_Process(void);
void ND_Cmd_OnRxByte(uint8_t ch);
const ND_Config_t* ND_Config_Get(void);
uint8_t ND_Config_GetSensorMask(void);

#ifdef __cplusplus
}
#endif

#endif /* ND_CFG_H */
