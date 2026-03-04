#include "ui_fault.h"

#include "stm32wlxx_hal.h"
#include <string.h>

#if defined(HAL_RTC_MODULE_ENABLED)
#include "stm32wlxx_hal_rtc_ex.h"
extern RTC_HandleTypeDef hrtc;
#define UI_FAULT_BKP_DR_MAGIC       (RTC_BKP_DR8)
#define UI_FAULT_BKP_DR_CP          (RTC_BKP_DR9)
#define UI_FAULT_BKP_DR_SEQ         (RTC_BKP_DR10)
#define UI_FAULT_BKP_DR_LINE        (RTC_BKP_DR11)
#define UI_FAULT_BKP_DR_A0          (RTC_BKP_DR12)
#define UI_FAULT_BKP_DR_A1          (RTC_BKP_DR13)
#define UI_FAULT_BKP_DR_NAME_HASH   (RTC_BKP_DR14)
#endif

#if defined(__GNUC__)
volatile UI_FaultLog_t g_ui_fault_log __attribute__((section(".noinit.ui_fault"), used));
#else
volatile UI_FaultLog_t g_ui_fault_log;
#endif

#if defined(__GNUC__)
#define UI_FAULT_NOINLINE __attribute__((noinline, used))
#else
#define UI_FAULT_NOINLINE
#endif

static void prv_copy_text(char* dst, uint32_t dst_len, const char* src)
{
    uint32_t i = 0;

    if (dst == NULL || dst_len == 0u)
    {
        return;
    }

    if (src == NULL)
    {
        dst[0] = '\0';
        return;
    }

    while ((i + 1u) < dst_len && src[i] != '\0')
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static uint32_t prv_hash_text(const char* src)
{
    uint32_t h = 2166136261u;

    if (src == NULL)
    {
        return 0u;
    }

    while (*src != '\0')
    {
        h ^= (uint32_t)(uint8_t)(*src++);
        h *= 16777619u;
    }
    return h;
}

static void prv_clear_current_runtime(void)
{
    g_ui_fault_log.last_line = 0u;
    g_ui_fault_log.last_a0 = 0u;
    g_ui_fault_log.last_a1 = 0u;
    g_ui_fault_log.last_name[0] = '\0';
    g_ui_fault_log.last_cmd[0] = '\0';
    g_ui_fault_log.wr = 0u;
    memset((void*)g_ui_fault_log.hist_name, 0, sizeof(g_ui_fault_log.hist_name));
    memset((void*)g_ui_fault_log.hist_line, 0, sizeof(g_ui_fault_log.hist_line));
    memset((void*)g_ui_fault_log.hist_a0, 0, sizeof(g_ui_fault_log.hist_a0));
    memset((void*)g_ui_fault_log.hist_a1, 0, sizeof(g_ui_fault_log.hist_a1));

    g_ui_fault_log.reset_flags_raw = 0u;
    g_ui_fault_log.reset_cause_mask = 0u;

    g_ui_fault_log.hardfault_seen = 0u;
    g_ui_fault_log.cfsr = 0u;
    g_ui_fault_log.hfsr = 0u;
    g_ui_fault_log.dfsr = 0u;
    g_ui_fault_log.afsr = 0u;
    g_ui_fault_log.bfar = 0u;
    g_ui_fault_log.mmfar = 0u;
    g_ui_fault_log.icsr = 0u;
    g_ui_fault_log.shcsr = 0u;
    g_ui_fault_log.msp = 0u;
    g_ui_fault_log.psp = 0u;
    g_ui_fault_log.lr = 0u;
    g_ui_fault_log.control = 0u;
    g_ui_fault_log.exc_return = 0u;
    g_ui_fault_log.stacked_r0 = 0u;
    g_ui_fault_log.stacked_r1 = 0u;
    g_ui_fault_log.stacked_r2 = 0u;
    g_ui_fault_log.stacked_r3 = 0u;
    g_ui_fault_log.stacked_r12 = 0u;
    g_ui_fault_log.stacked_lr = 0u;
    g_ui_fault_log.stacked_pc = 0u;
    g_ui_fault_log.stacked_xpsr = 0u;

    g_ui_fault_log.checkpoint_id = UI_CP_NONE;
    g_ui_fault_log.checkpoint_seq = 0u;
    g_ui_fault_log.checkpoint_name_hash = 0u;
}

static void prv_copy_prev_snapshot(const UI_FaultLog_t* snap)
{
    if (snap == NULL)
    {
        return;
    }

    g_ui_fault_log.prev_valid = 1u;
    g_ui_fault_log.prev_boot_count = snap->boot_count;
    g_ui_fault_log.prev_last_line = snap->last_line;
    g_ui_fault_log.prev_last_a0 = snap->last_a0;
    g_ui_fault_log.prev_last_a1 = snap->last_a1;
    prv_copy_text((char*)g_ui_fault_log.prev_last_name, UI_FAULT_NAME_LEN, (const char*)snap->last_name);
    prv_copy_text((char*)g_ui_fault_log.prev_last_cmd, UI_FAULT_CMD_LEN, (const char*)snap->last_cmd);
    g_ui_fault_log.prev_checkpoint_id = snap->checkpoint_id;
    g_ui_fault_log.prev_reset_flags_raw = snap->reset_flags_raw;
    g_ui_fault_log.prev_reset_cause_mask = snap->reset_cause_mask;
    g_ui_fault_log.prev_hardfault_seen = snap->hardfault_seen;
    g_ui_fault_log.prev_cfsr = snap->cfsr;
    g_ui_fault_log.prev_hfsr = snap->hfsr;
    g_ui_fault_log.prev_stacked_pc = snap->stacked_pc;
    g_ui_fault_log.prev_stacked_lr = snap->stacked_lr;
}

#if defined(HAL_RTC_MODULE_ENABLED)
static void prv_bkp_write_checkpoint(uint32_t checkpoint_id, uint32_t seq, uint32_t line, uint32_t a0, uint32_t a1, uint32_t name_hash)
{
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_MAGIC, UI_FAULT_BKP_MAGIC);
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_CP, checkpoint_id);
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_SEQ, seq);
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_LINE, line);
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_A0, a0);
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_A1, a1);
    HAL_RTCEx_BKUPWrite(&hrtc, UI_FAULT_BKP_DR_NAME_HASH, name_hash);
}

static void prv_bkp_restore(void)
{
    uint32_t magic = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_MAGIC);

    if (magic != UI_FAULT_BKP_MAGIC)
    {
        g_ui_fault_log.bkp_valid = 0u;
        return;
    }

    g_ui_fault_log.bkp_valid = 1u;
    g_ui_fault_log.bkp_checkpoint_id = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_CP);
    g_ui_fault_log.bkp_seq = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_SEQ);
    g_ui_fault_log.bkp_line = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_LINE);
    g_ui_fault_log.bkp_a0 = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_A0);
    g_ui_fault_log.bkp_a1 = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_A1);
    g_ui_fault_log.bkp_name_hash = HAL_RTCEx_BKUPRead(&hrtc, UI_FAULT_BKP_DR_NAME_HASH);
}
#else
static void prv_bkp_write_checkpoint(uint32_t checkpoint_id, uint32_t seq, uint32_t line, uint32_t a0, uint32_t a1, uint32_t name_hash)
{
    (void)checkpoint_id;
    (void)seq;
    (void)line;
    (void)a0;
    (void)a1;
    (void)name_hash;
}

static void prv_bkp_restore(void)
{
    g_ui_fault_log.bkp_valid = 0u;
    g_ui_fault_log.bkp_checkpoint_id = 0u;
    g_ui_fault_log.bkp_seq = 0u;
    g_ui_fault_log.bkp_line = 0u;
    g_ui_fault_log.bkp_a0 = 0u;
    g_ui_fault_log.bkp_a1 = 0u;
    g_ui_fault_log.bkp_name_hash = 0u;
}
#endif

void UI_Fault_Init(void)
{
    UI_FaultLog_t snap;
    uint32_t boot_count = 1u;
    uint8_t have_snap = (g_ui_fault_log.magic == UI_FAULT_MAGIC) ? 1u : 0u;

    if (have_snap != 0u)
    {
        memcpy(&snap, (const void*)&g_ui_fault_log, sizeof(snap));
        boot_count = snap.boot_count + 1u;
    }

    memset((void*)&g_ui_fault_log, 0, sizeof(g_ui_fault_log));
    g_ui_fault_log.magic = UI_FAULT_MAGIC;
    g_ui_fault_log.boot_count = boot_count;

    if (have_snap != 0u)
    {
        prv_copy_prev_snapshot(&snap);
    }

    prv_bkp_restore();

    if ((have_snap == 0u) && (g_ui_fault_log.bkp_valid != 0u))
    {
        g_ui_fault_log.prev_valid = 1u;
        g_ui_fault_log.prev_last_line = g_ui_fault_log.bkp_line;
        g_ui_fault_log.prev_last_a0 = g_ui_fault_log.bkp_a0;
        g_ui_fault_log.prev_last_a1 = g_ui_fault_log.bkp_a1;
        prv_copy_text((char*)g_ui_fault_log.prev_last_name, UI_FAULT_NAME_LEN, "BKP_ONLY");
        g_ui_fault_log.prev_checkpoint_id = g_ui_fault_log.bkp_checkpoint_id;
    }

    prv_clear_current_runtime();
    UI_Fault_CaptureResetFlags();

    if ((have_snap != 0u) || (g_ui_fault_log.bkp_valid != 0u) || (g_ui_fault_log.reset_cause_mask != 0u))
    {
        UI_Fault_Checkpoint(UI_CP_BOOT_RECOVERED,
                            "BOOT_RECOV",
                            0u,
                            g_ui_fault_log.reset_cause_mask,
                            ((have_snap != 0u) ? 1u : 0u) | ((g_ui_fault_log.bkp_valid != 0u) ? 2u : 0u));
        UI_Fault_Bp_ResetRecovered();
    }
}

void UI_Fault_ClearRuntime(void)
{
    prv_clear_current_runtime();
}

void UI_Fault_Mark(const char* name, uint32_t line, uint32_t a0, uint32_t a1)
{
    uint32_t idx = g_ui_fault_log.wr % UI_FAULT_TRACE_DEPTH;

    g_ui_fault_log.magic = UI_FAULT_MAGIC;
    g_ui_fault_log.last_line = line;
    g_ui_fault_log.last_a0 = a0;
    g_ui_fault_log.last_a1 = a1;
    g_ui_fault_log.checkpoint_name_hash = prv_hash_text(name);

    g_ui_fault_log.checkpoint_seq++;

    prv_copy_text((char*)g_ui_fault_log.last_name, UI_FAULT_NAME_LEN, name);
    prv_copy_text((char*)g_ui_fault_log.hist_name[idx], UI_FAULT_NAME_LEN, name);
    g_ui_fault_log.hist_line[idx] = line;
    g_ui_fault_log.hist_a0[idx] = a0;
    g_ui_fault_log.hist_a1[idx] = a1;
    g_ui_fault_log.wr = idx + 1u;
}

void UI_Fault_MarkCmd(const char* cmd)
{
    prv_copy_text((char*)g_ui_fault_log.last_cmd, UI_FAULT_CMD_LEN, cmd);
}

void UI_Fault_Checkpoint(uint32_t checkpoint_id, const char* name, uint32_t line, uint32_t a0, uint32_t a1)
{
    UI_Fault_Mark(name, line, a0, a1);
    g_ui_fault_log.checkpoint_id = checkpoint_id;
    g_ui_fault_log.checkpoint_seq = g_ui_fault_log.checkpoint_seq;
    prv_bkp_write_checkpoint(checkpoint_id,
                             g_ui_fault_log.checkpoint_seq,
                             line,
                             a0,
                             a1,
                             g_ui_fault_log.checkpoint_name_hash);
}

void UI_Fault_CaptureResetFlags(void)
{
    uint32_t csr = RCC->CSR;
    uint32_t mask = 0u;

#if defined(RCC_CSR_LPWRRSTF)
    if ((csr & RCC_CSR_LPWRRSTF) != 0u) { mask |= UI_RESET_CAUSE_LPWR; }
#endif
#if defined(RCC_CSR_WWDGRSTF)
    if ((csr & RCC_CSR_WWDGRSTF) != 0u) { mask |= UI_RESET_CAUSE_WWDG; }
#endif
#if defined(RCC_CSR_IWDGRSTF)
    if ((csr & RCC_CSR_IWDGRSTF) != 0u) { mask |= UI_RESET_CAUSE_IWDG; }
#endif
#if defined(RCC_CSR_SFTRSTF)
    if ((csr & RCC_CSR_SFTRSTF) != 0u) { mask |= UI_RESET_CAUSE_SFT; }
#endif
#if defined(RCC_CSR_PORRSTF)
    if ((csr & RCC_CSR_PORRSTF) != 0u) { mask |= UI_RESET_CAUSE_POR; }
#endif
#if defined(RCC_CSR_PINRSTF)
    if ((csr & RCC_CSR_PINRSTF) != 0u) { mask |= UI_RESET_CAUSE_PIN; }
#endif
#if defined(RCC_CSR_BORRSTF)
    if ((csr & RCC_CSR_BORRSTF) != 0u) { mask |= UI_RESET_CAUSE_BOR; }
#endif

    g_ui_fault_log.magic = UI_FAULT_MAGIC;
    g_ui_fault_log.reset_flags_raw = csr;
    g_ui_fault_log.reset_cause_mask = mask;

#if defined(__HAL_RCC_CLEAR_RESET_FLAGS)
    __HAL_RCC_CLEAR_RESET_FLAGS();
#elif defined(RCC_CSR_RMVF)
    SET_BIT(RCC->CSR, RCC_CSR_RMVF);
#endif
}

void UI_Fault_CaptureHardFault(void)
{
    g_ui_fault_log.magic = UI_FAULT_MAGIC;
    g_ui_fault_log.hardfault_seen++;
    g_ui_fault_log.checkpoint_id = UI_CP_HARDFAULT;
    g_ui_fault_log.checkpoint_name_hash = prv_hash_text("HARDFAULT");
    prv_copy_text((char*)g_ui_fault_log.last_name, UI_FAULT_NAME_LEN, "HARDFAULT");
    g_ui_fault_log.cfsr = SCB->CFSR;
    g_ui_fault_log.hfsr = SCB->HFSR;
    g_ui_fault_log.dfsr = SCB->DFSR;
    g_ui_fault_log.afsr = SCB->AFSR;
    g_ui_fault_log.bfar = SCB->BFAR;
    g_ui_fault_log.mmfar = SCB->MMFAR;
    g_ui_fault_log.icsr = SCB->ICSR;
    g_ui_fault_log.shcsr = SCB->SHCSR;
    g_ui_fault_log.msp = __get_MSP();
    g_ui_fault_log.psp = __get_PSP();
    {
        uint32_t lr_reg = 0u;
        __asm volatile ("MOV %0, LR" : "=r"(lr_reg));
        g_ui_fault_log.lr = lr_reg;
    }
    g_ui_fault_log.control = __get_CONTROL();
    UI_Fault_Bp_HardFault();
}

void UI_Fault_CaptureHardFaultStack(uint32_t* sp, uint32_t exc_return)
{
    UI_Fault_CaptureHardFault();

    g_ui_fault_log.exc_return = exc_return;

    if (sp != NULL)
    {
        g_ui_fault_log.stacked_r0   = sp[0];
        g_ui_fault_log.stacked_r1   = sp[1];
        g_ui_fault_log.stacked_r2   = sp[2];
        g_ui_fault_log.stacked_r3   = sp[3];
        g_ui_fault_log.stacked_r12  = sp[4];
        g_ui_fault_log.stacked_lr   = sp[5];
        g_ui_fault_log.stacked_pc   = sp[6];
        g_ui_fault_log.stacked_xpsr = sp[7];
    }
}

__attribute__((naked)) void UI_Fault_HardFaultEntry(void)
{
    __asm volatile
    (
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "b UI_Fault_CaptureHardFaultStack\n"
    );
}

UI_FAULT_NOINLINE void UI_Fault_Bp_ResetRecovered(void) { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_StopEnter(void)      { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_BleBtOn(void)        { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_BleUartInit(void)    { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_BleAtReset(void)     { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_UartError(void)      { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_GwKeepalive(void)    { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_GwWakeCb(void)       { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_GwBeaconSend(void)   { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_GwRxArm(void)        { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_GwRxDone(void)       { __asm volatile ("" ::: "memory"); }
UI_FAULT_NOINLINE void UI_Fault_Bp_HardFault(void)      { __asm volatile ("" ::: "memory"); }
