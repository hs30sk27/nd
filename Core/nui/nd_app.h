/*
 * nd_app.h
 *
 * Node 동작(비콘 수신 + 센서 체크 + 데이터 송신 스케줄러)
 *
 * 요구사항 핵심:
 *  - 전원 인가 후 5분 10초 동안 비콘 채널 RX 준비
 *  - 비콘을 받지 못하면 센서 체크/송신 동작하지 않음
 *  - 비콘 수신 시 시간/테스트 설정 반영
 *  - 정상모드: 매 시간 00:06 센서 체크, 01:00부터 node별 2초 간격 송신
 *  - 테스트모드: 1분 주기, 00:06 센서 체크, 30초부터 node별 2초 간격 송신
 *  - OP_KEY: 센서 측정 후 BLE ON + ASCII 데이터 송신, 동작중 LED1 ON
 *  - LoRa 동작 종료 시 Radio.Sleep()
 */

#ifndef ND_APP_H
#define ND_APP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ND_App_Init(void);

/* 단일 task 모드에서 UI_MAIN에서 호출하여 이벤트를 처리 */
void ND_App_Process(void);

/* BLE 세션 시작/종료 시 ND 런타임을 정지/재개 */
void ND_App_OnBleSessionStart(void);
void ND_App_OnBleSessionEnd(void);

/* 비콘 탐색 중 키 입력으로 탐색을 중지하고 stop mode로 전환 */
bool ND_App_StopBeaconSearchAndEnterStop(void);

/* Radio event에서 호출 (subghz_phy_app.c USER CODE에 삽입) */
void ND_Radio_OnTxDone(void);
void ND_Radio_OnTxTimeout(void);
void ND_Radio_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void ND_Radio_OnRxTimeout(void);
void ND_Radio_OnRxError(void);

#ifdef __cplusplus
}
#endif

#endif /* ND_APP_H */
