/*
 * nd_sensors.h
 *
 * Node 센서 체크 모듈
 *
 * 요구사항:
 *  1) batt level은 ADC가 아니라 BATT_LVL GPIO 입력(1/0)으로 판정
 *  2) 내부 온도(ADC): N회 측정 -> sort -> 양끝 trim 후 중간값 평균
 *  3) ICM20948: WHO_AM_I 확인 실패 시 스킵(0xFFFF 처리)
 *     100회 측정 -> sort -> 양끝 20개 trim 후 중간 60개 평균
 *  4) LTC2450(외부 ADC): N회 측정 -> sort -> 양끝 trim 후 중간값 평균
 *  5) PULSE COUNT: PULSE_IN EXTI 발생 시 ++
 *
 * NOTE:
 *  - 실제 하드웨어 배선/전원(ADC_EN) 조건에 맞게 딜레이/레지스터 값은 조정 가능.
 *  - 센서 측정은 메인 컨텍스트에서 수행(인터럽트에서 delay 금지).
 */

#ifndef ND_SENSORS_H
#define ND_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t  batt_lvl;   /* BATT_LVL GPIO 기반: 1=normal, 0=low */
    int8_t   temp_c;     /* -50..100'C, UI_NODE_TEMP_INVALID_C=internal invalid */
    int16_t  x;
    int16_t  y;
    int16_t  z;
    uint16_t adc;        /* LTC2450 raw avg */
    uint32_t pulse_cnt;  /* PULSE_IN 누적 카운트 */
} ND_SensorResult_t;

void ND_Sensors_Init(void);

/* 센서 전체 측정(최대 50초 이내 권장)
 * sensor_en_mask: bit0=ICM20948, bit1=ADC, bit2=PULSE */
bool ND_Sensors_MeasureAll(ND_SensorResult_t* out, uint8_t sensor_en_mask);

#ifdef __cplusplus
}
#endif

#endif /* ND_SENSORS_H */
