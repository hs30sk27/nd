#ifndef UI_FAULT_H
#define UI_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_FAULT_TRACE_DEPTH   (16u)
#define UI_FAULT_NAME_LEN      (24u)
#define UI_FAULT_CMD_LEN       (64u)
#define UI_FAULT_MAGIC         (0x55494654u) /* 'UIFT' */
#define UI_FAULT_BKP_MAGIC     (0x55494642u) /* 'UIFB' */

#define UI_RESET_CAUSE_LPWR      (1u << 0)
#define UI_RESET_CAUSE_WWDG      (1u << 1)
#define UI_RESET_CAUSE_IWDG      (1u << 2)
#define UI_RESET_CAUSE_SFT       (1u << 3)
#define UI_RESET_CAUSE_POR       (1u << 4)
#define UI_RESET_CAUSE_PIN       (1u << 5)
#define UI_RESET_CAUSE_BOR       (1u << 6)

typedef enum
{
    UI_CP_NONE             = 0x0000u,
    UI_CP_BOOT_RECOVERED   = 0x0101u,
    UI_CP_BOOT_INIT_BEGIN  = 0x0102u,
    UI_CP_BOOT_INIT_DONE   = 0x0103u,
    UI_CP_STOP_PRE         = 0x0110u,
    UI_CP_STOP_SAVE        = 0x0111u,
    UI_CP_BLE_ENABLE       = 0x0201u,
    UI_CP_BLE_UART_INIT    = 0x0202u,
    UI_CP_BLE_AT_RESET     = 0x0203u,
    UI_CP_BLE_DISABLE      = 0x0204u,
    UI_CP_BLE_TIMEOUT      = 0x0205u,
    UI_CP_UART_ERROR       = 0x0210u,
    UI_CP_GW_INIT          = 0x0301u,
    UI_CP_GW_KEEPALIVE     = 0x0302u,
    UI_CP_GW_WAKE_CB       = 0x0303u,
    UI_CP_GW_BEACON_SEND   = 0x0304u,
    UI_CP_GW_RX_ARM        = 0x0305u,
    UI_CP_GW_RX_DONE       = 0x0306u,
    UI_CP_HARDFAULT        = 0x0F01u,
} UI_FaultCheckpointId_t;

typedef struct
{
    uint32_t magic;
    uint32_t boot_count;

    uint32_t last_line;
    uint32_t last_a0;
    uint32_t last_a1;
    char     last_name[UI_FAULT_NAME_LEN];
    char     last_cmd[UI_FAULT_CMD_LEN];

    uint32_t wr;
    char     hist_name[UI_FAULT_TRACE_DEPTH][UI_FAULT_NAME_LEN];
    uint32_t hist_line[UI_FAULT_TRACE_DEPTH];
    uint32_t hist_a0[UI_FAULT_TRACE_DEPTH];
    uint32_t hist_a1[UI_FAULT_TRACE_DEPTH];

    uint32_t reset_flags_raw;
    uint32_t reset_cause_mask;

    uint32_t hardfault_seen;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t icsr;
    uint32_t shcsr;
    uint32_t msp;
    uint32_t psp;
    uint32_t lr;
    uint32_t control;
    uint32_t exc_return;
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_xpsr;

    uint32_t checkpoint_id;
    uint32_t checkpoint_seq;
    uint32_t checkpoint_name_hash;

    uint32_t prev_valid;
    uint32_t prev_boot_count;
    uint32_t prev_last_line;
    uint32_t prev_last_a0;
    uint32_t prev_last_a1;
    char     prev_last_name[UI_FAULT_NAME_LEN];
    char     prev_last_cmd[UI_FAULT_CMD_LEN];
    uint32_t prev_checkpoint_id;
    uint32_t prev_reset_flags_raw;
    uint32_t prev_reset_cause_mask;
    uint32_t prev_hardfault_seen;
    uint32_t prev_cfsr;
    uint32_t prev_hfsr;
    uint32_t prev_stacked_pc;
    uint32_t prev_stacked_lr;

    uint32_t bkp_valid;
    uint32_t bkp_checkpoint_id;
    uint32_t bkp_seq;
    uint32_t bkp_line;
    uint32_t bkp_a0;
    uint32_t bkp_a1;
    uint32_t bkp_name_hash;
} UI_FaultLog_t;

extern volatile UI_FaultLog_t g_ui_fault_log;

void UI_Fault_Init(void);
void UI_Fault_ClearRuntime(void);
void UI_Fault_Mark(const char* name, uint32_t line, uint32_t a0, uint32_t a1);
void UI_Fault_MarkCmd(const char* cmd);
void UI_Fault_Checkpoint(uint32_t checkpoint_id, const char* name, uint32_t line, uint32_t a0, uint32_t a1);
void UI_Fault_CaptureResetFlags(void);
void UI_Fault_CaptureHardFault(void);
void UI_Fault_CaptureHardFaultStack(uint32_t* sp, uint32_t exc_return);
void UI_Fault_HardFaultEntry(void);

void UI_Fault_Bp_ResetRecovered(void);
void UI_Fault_Bp_StopEnter(void);
void UI_Fault_Bp_BleBtOn(void);
void UI_Fault_Bp_BleUartInit(void);
void UI_Fault_Bp_BleAtReset(void);
void UI_Fault_Bp_UartError(void);
void UI_Fault_Bp_GwKeepalive(void);
void UI_Fault_Bp_GwWakeCb(void);
void UI_Fault_Bp_GwBeaconSend(void);
void UI_Fault_Bp_GwRxArm(void);
void UI_Fault_Bp_GwRxDone(void);
void UI_Fault_Bp_HardFault(void);

#define UI_FAULT_MARK(name, a0, a1) \
    UI_Fault_Mark((name), (uint32_t)__LINE__, (uint32_t)(a0), (uint32_t)(a1))

#define UI_FAULT_CP(id, name, a0, a1) \
    UI_Fault_Checkpoint((uint32_t)(id), (name), (uint32_t)__LINE__, (uint32_t)(a0), (uint32_t)(a1))

#ifdef __cplusplus
}
#endif

#endif /* UI_FAULT_H */
