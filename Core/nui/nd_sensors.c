#include "nd_sensors.h"
#include "ui_types.h"
#include "ui_conf.h"
#include "ui_gpio.h"
#include "main.h"
#include "stm32wlxx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* 프로젝트 핸들 */
extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;

/* main.c에 생성된 Init 함수(Stop wake 후 필요할 때만 호출) */
extern void MX_ADC_Init(void);
extern void MX_SPI1_Init(void);

/* -------------------------------------------------------------------------- */
/* 내부 온도 측정용 ADC 프로파일                                              */
/* -------------------------------------------------------------------------- */
#if defined(ADC_SAMPLETIME_640CYCLES_5)
#define UI_ND_ADC_INTERNAL_COMMON_SAMPLE    (ADC_SAMPLETIME_640CYCLES_5)
#elif defined(ADC_SAMPLETIME_247CYCLES_5)
#define UI_ND_ADC_INTERNAL_COMMON_SAMPLE    (ADC_SAMPLETIME_247CYCLES_5)
#elif defined(ADC_SAMPLETIME_160CYCLES_5)
#define UI_ND_ADC_INTERNAL_COMMON_SAMPLE    (ADC_SAMPLETIME_160CYCLES_5)
#elif defined(ADC_SAMPLETIME_92CYCLES_5)
#define UI_ND_ADC_INTERNAL_COMMON_SAMPLE    (ADC_SAMPLETIME_92CYCLES_5)
#elif defined(ADC_SAMPLETIME_47CYCLES_5)
#define UI_ND_ADC_INTERNAL_COMMON_SAMPLE    (ADC_SAMPLETIME_47CYCLES_5)
#else
#define UI_ND_ADC_INTERNAL_COMMON_SAMPLE    (UI_ADC_SAMPLINGTIME)
#endif

#if defined(ADC_CLOCK_ASYNC_DIV64)
#define UI_ND_ADC_INTERNAL_CLOCK_PRESCALER  (ADC_CLOCK_ASYNC_DIV64)
#elif defined(ADC_CLOCK_SYNC_PCLK_DIV4)
#define UI_ND_ADC_INTERNAL_CLOCK_PRESCALER  (ADC_CLOCK_SYNC_PCLK_DIV4)
#else
#define UI_ND_ADC_INTERNAL_CLOCK_PRESCALER  (ADC_CLOCK_SYNC_PCLK_DIV1)
#endif

#define UI_ND_TEMP_VREF_SAMPLE_COUNT        (5u)
#define UI_ND_TEMP_VREF_TRIM_COUNT          (1u)
#define UI_ND_TEMP_RAW_MIN_VALID            (16u)
#define UI_ND_TEMP_RAW_MAX_VALID            (4080u)
#define UI_ND_VDD_MIN_VALID_MV              (1800u)
#define UI_ND_VDD_MAX_VALID_MV              (3600u)
#define UI_ND_TEMP_STARTUP_DELAY_MS         (2u)

static int8_t s_last_valid_temp_c = UI_NODE_TEMP_INVALID_C;
static uint8_t s_last_valid_batt_lvl = UI_NODE_BATT_LVL_INVALID;

/* -------------------------------------------------------------------------- */
/* 유틸: 정렬 + 트림 평균                                                     */
/* -------------------------------------------------------------------------- */
static void prv_sort_u16(uint16_t* a, uint16_t n)
{
    for (uint16_t i = 1u; i < n; i++) {
        uint16_t key = a[i];
        int j = (int)i - 1;
        while ((j >= 0) && (a[j] > key)) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

static void prv_sort_i16(int16_t* a, uint16_t n)
{
    for (uint16_t i = 1u; i < n; i++) {
        int16_t key = a[i];
        int j = (int)i - 1;
        while ((j >= 0) && (a[j] > key)) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

static uint16_t prv_trimmed_mean_u16(uint16_t* a, uint16_t n, uint16_t trim_each_side)
{
    if (n == 0u) {
        return 0xFFFFu;
    }

    prv_sort_u16(a, n);

    uint16_t start = trim_each_side;
    uint16_t end = (uint16_t)(n - trim_each_side);
    if (end <= start) {
        return a[n / 2u];
    }

    uint32_t sum = 0u;
    uint32_t cnt = 0u;
    for (uint16_t i = start; i < end; i++) {
        sum += a[i];
        cnt++;
    }

    return (cnt == 0u) ? a[n / 2u] : (uint16_t)(sum / cnt);
}

static int16_t prv_trimmed_mean_i16(int16_t* a, uint16_t n, uint16_t trim_each_side)
{
    if (n == 0u) {
        return (int16_t)0xFFFFu;
    }

    prv_sort_i16(a, n);

    uint16_t start = trim_each_side;
    uint16_t end = (uint16_t)(n - trim_each_side);
    if (end <= start) {
        return a[n / 2u];
    }

    int32_t sum = 0;
    uint32_t cnt = 0u;
    for (uint16_t i = start; i < end; i++) {
        sum += a[i];
        cnt++;
    }

    return (cnt == 0u) ? a[n / 2u] : (int16_t)(sum / (int32_t)cnt);
}

/* -------------------------------------------------------------------------- */
/* ADC_EN 전원 스위치                                                         */
/* -------------------------------------------------------------------------- */
static void prv_set_adc_en(bool on)
{
#if defined(ADC_EN_Pin)
    HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

/* -------------------------------------------------------------------------- */
/* Init Ensure (Stop wake 이후 필요한 주변장치만 Init)                        */
/* -------------------------------------------------------------------------- */
static void prv_ensure_adc_init(void)
{
#if defined(HAL_ADC_MODULE_ENABLED)
    if (hadc.State == HAL_ADC_STATE_RESET) {
        MX_ADC_Init();
    }
#endif
}

static void prv_ensure_spi_init(void)
{
#if defined(HAL_SPI_MODULE_ENABLED)
    if (hspi1.State == HAL_SPI_STATE_RESET) {
        MX_SPI1_Init();
    }
#endif
}

static bool prv_prepare_internal_adc_profile(void)
{
#if defined(HAL_ADC_MODULE_ENABLED)
    bool changed = false;

    prv_ensure_adc_init();

    if (hadc.Init.ClockPrescaler != UI_ND_ADC_INTERNAL_CLOCK_PRESCALER) {
        hadc.Init.ClockPrescaler = UI_ND_ADC_INTERNAL_CLOCK_PRESCALER;
        changed = true;
    }

    if (hadc.Init.SamplingTimeCommon1 != UI_ND_ADC_INTERNAL_COMMON_SAMPLE) {
        hadc.Init.SamplingTimeCommon1 = UI_ND_ADC_INTERNAL_COMMON_SAMPLE;
        changed = true;
    }

    if (hadc.Init.SamplingTimeCommon2 != UI_ND_ADC_INTERNAL_COMMON_SAMPLE) {
        hadc.Init.SamplingTimeCommon2 = UI_ND_ADC_INTERNAL_COMMON_SAMPLE;
        changed = true;
    }

    if (changed) {
        (void)HAL_ADC_DeInit(&hadc);
        if (HAL_ADC_Init(&hadc) != HAL_OK) {
            return false;
        }
    }

    /* STM32WLE5 internal channels need ADC self-calibration after re-init. */
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
        return false;
    }

    HAL_Delay(UI_ND_TEMP_STARTUP_DELAY_MS);
    return true;
#else
    return false;
#endif
}

/* -------------------------------------------------------------------------- */
/* 내부 ADC (VREFINT/TEMPSENSOR)                                              */
/* -------------------------------------------------------------------------- */
static bool prv_adc_read_internal(uint32_t channel, uint16_t* out_raw)
{
#if defined(HAL_ADC_MODULE_ENABLED)
    ADC_ChannelConfTypeDef sConfig = {0};

    if (out_raw == NULL) {
        return false;
    }

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
#if defined(ADC_SAMPLINGTIME_COMMON_1)
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
#else
    sConfig.SamplingTime = UI_ND_ADC_INTERNAL_COMMON_SAMPLE;
#endif

    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        return false;
    }

    if (HAL_ADC_Start(&hadc) != HAL_OK) {
        return false;
    }

    if (HAL_ADC_PollForConversion(&hadc, 50u) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc);
        return false;
    }

    *out_raw = (uint16_t)HAL_ADC_GetValue(&hadc);
    (void)HAL_ADC_Stop(&hadc);
    return true;
#else
    (void)channel;
    (void)out_raw;
    return false;
#endif
}

static bool prv_read_vdd_mv_filtered(uint16_t* out_vdd_mv)
{
#if defined(ADC_CHANNEL_VREFINT) && defined(__HAL_ADC_CALC_VREFANALOG_VOLTAGE)
    uint16_t samples[UI_ND_TEMP_VREF_SAMPLE_COUNT];

    if (out_vdd_mv == NULL) {
        return false;
    }

    for (uint32_t i = 0u; i < UI_ND_TEMP_VREF_SAMPLE_COUNT; i++) {
        uint16_t raw = 0u;
        if (!prv_adc_read_internal(ADC_CHANNEL_VREFINT, &raw)) {
            return false;
        }
        if ((raw < UI_ND_TEMP_RAW_MIN_VALID) || (raw > UI_ND_TEMP_RAW_MAX_VALID)) {
            return false;
        }
        samples[i] = raw;
        HAL_Delay(1u);
    }

    uint16_t raw_mid = prv_trimmed_mean_u16(samples,
                                            UI_ND_TEMP_VREF_SAMPLE_COUNT,
                                            UI_ND_TEMP_VREF_TRIM_COUNT);
    uint32_t vdd_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw_mid, ADC_RESOLUTION_12B);
    if ((vdd_mv < UI_ND_VDD_MIN_VALID_MV) || (vdd_mv > UI_ND_VDD_MAX_VALID_MV)) {
        return false;
    }

    *out_vdd_mv = (uint16_t)vdd_mv;
    return true;
#else
    (void)out_vdd_mv;
    return false;
#endif
}

static bool prv_read_temp_x10_internal(int16_t* out_temp_x10, uint16_t* out_vdd_mv)
{
#if defined(ADC_CHANNEL_TEMPSENSOR) && defined(__HAL_ADC_CALC_TEMPERATURE)
    uint16_t samples[UI_NODE_TEMP_SAMPLE_COUNT];
    uint16_t vdd_mv = 0u;

    if (out_temp_x10 == NULL) {
        return false;
    }
    if (out_vdd_mv != NULL) {
        *out_vdd_mv = 0u;
    }

    /*
     * MCU 내부 온도는 외부 센서와 무관하므로 가장 먼저 읽는다.
     * ND 보드는 main.c 기본 ADC 설정이 빠른 쪽(짧은 sample time / 빠른 clock)이라
     * 내부 채널(VREFINT/TEMPSENSOR)에서 과소 샘플링이 나올 수 있으므로, 여기서
     * 내부 채널 전용 프로파일로 재설정한 뒤 측정한다.
     */
    prv_set_adc_en(false);
    HAL_Delay(UI_ND_TEMP_STARTUP_DELAY_MS);

    if (!prv_prepare_internal_adc_profile()) {
        return false;
    }

    if (!prv_read_vdd_mv_filtered(&vdd_mv)) {
        return false;
    }
    if (out_vdd_mv != NULL) {
        *out_vdd_mv = vdd_mv;
    }

    for (uint32_t i = 0u; i < UI_NODE_TEMP_WARMUP_COUNT; i++) {
        uint16_t raw_dummy = 0u;
        if (!prv_adc_read_internal(ADC_CHANNEL_TEMPSENSOR, &raw_dummy)) {
            return false;
        }
        HAL_Delay(UI_NODE_TEMP_WARMUP_DELAY_MS);
    }

    for (uint32_t i = 0u; i < UI_NODE_TEMP_SAMPLE_COUNT; i++) {
        uint16_t raw = 0u;
        if (!prv_adc_read_internal(ADC_CHANNEL_TEMPSENSOR, &raw)) {
            return false;
        }
        if ((raw < UI_ND_TEMP_RAW_MIN_VALID) || (raw > UI_ND_TEMP_RAW_MAX_VALID)) {
            return false;
        }
        samples[i] = raw;
        HAL_Delay(2u);
    }

    uint16_t raw_mid = prv_trimmed_mean_u16(samples,
                                            UI_NODE_TEMP_SAMPLE_COUNT,
                                            UI_NODE_TEMP_TRIM_COUNT);
    if ((raw_mid < UI_ND_TEMP_RAW_MIN_VALID) || (raw_mid > UI_ND_TEMP_RAW_MAX_VALID)) {
        return false;
    }

    {
        int32_t temp_c = __HAL_ADC_CALC_TEMPERATURE(vdd_mv, raw_mid, ADC_RESOLUTION_12B);
        *out_temp_x10 = (int16_t)(temp_c * 10);
    }

    return true;
#else
    (void)out_temp_x10;
    (void)out_vdd_mv;
    return false;
#endif
}

static int8_t prv_clamp_temp_c_i16(int16_t temp_c)
{
    if (temp_c < (int16_t)UI_NODE_TEMP_MIN_C) {
        temp_c = (int16_t)UI_NODE_TEMP_MIN_C;
    }
    if (temp_c > (int16_t)UI_NODE_TEMP_MAX_C) {
        temp_c = (int16_t)UI_NODE_TEMP_MAX_C;
    }
    return (int8_t)temp_c;
}

static bool prv_temp_x10_to_temp_c_checked(int16_t temp_x10, int8_t* out_temp_c)
{
    int16_t temp_c = 0;

    if (out_temp_c == NULL) {
        return false;
    }

    if ((uint16_t)temp_x10 == 0xFFFFu) {
        return false;
    }

    if (temp_x10 >= 0) {
        temp_c = (int16_t)((temp_x10 + 5) / 10);
    } else {
        temp_c = (int16_t)((temp_x10 - 5) / 10);
    }

    if ((temp_c < (int16_t)UI_NODE_TEMP_MIN_C) || (temp_c > (int16_t)UI_NODE_TEMP_MAX_C)) {
        return false;
    }

    *out_temp_c = (int8_t)temp_c;
    return true;
}

static int8_t prv_apply_temp_offset_c(int8_t temp_c)
{
    if (temp_c == UI_NODE_TEMP_INVALID_C) {
        return UI_NODE_TEMP_INVALID_C;
    }
    return prv_clamp_temp_c_i16((int16_t)temp_c + (int16_t)UI_NODE_TEMP_OFFSET_C);
}

static bool prv_read_batt_lvl_gpio(uint8_t* out_batt_lvl)
{
#if defined(BATT_LVL_Pin) && defined(BATT_LVL_GPIO_Port)
    if (out_batt_lvl == NULL) {
        return false;
    }

    *out_batt_lvl = (HAL_GPIO_ReadPin(BATT_LVL_GPIO_Port, BATT_LVL_Pin) == UI_BATT_LVL_NORMAL_GPIO_STATE)
                  ? UI_NODE_BATT_LVL_NORMAL
                  : UI_NODE_BATT_LVL_LOW;
    return true;
#else
    (void)out_batt_lvl;
    return false;
#endif
}

/* -------------------------------------------------------------------------- */
/* ICM20948 (SPI)                                                             */
/* -------------------------------------------------------------------------- */
#if defined(ICM20948_CS_Pin)
static void prv_icm_cs(bool on) /* on=true: CS low */
{
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port,
                      ICM20948_CS_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool prv_icm_spi_read(uint8_t reg, uint8_t* buf, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80u);

    prv_icm_cs(true);
    if (HAL_SPI_Transmit(&hspi1, &addr, 1u, 50u) != HAL_OK) {
        prv_icm_cs(false);
        return false;
    }
    if (HAL_SPI_Receive(&hspi1, buf, len, 50u) != HAL_OK) {
        prv_icm_cs(false);
        return false;
    }
    prv_icm_cs(false);
    return true;
}

static bool prv_icm_spi_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };

    prv_icm_cs(true);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, 2u, 50u);
    prv_icm_cs(false);
    return (st == HAL_OK);
}

static bool prv_icm_select_bank(uint8_t bank)
{
    return prv_icm_spi_write(0x7Fu, (uint8_t)(bank << 4));
}

static bool prv_icm_wakeup(void)
{
    if (!prv_icm_select_bank(0u)) {
        return false;
    }
    if (!prv_icm_spi_write(0x06u, 0x01u)) {
        return false;
    }
    HAL_Delay(10u);
    return true;
}

static bool prv_icm_sleep(void)
{
    if (!prv_icm_select_bank(0u)) {
        return false;
    }
    /* PWR_MGMT_1[6]=1 sleep, keep best available clock select to avoid undefined state. */
    return prv_icm_spi_write(0x06u, 0x41u);
}

static bool prv_icm_check_whoami(void)
{
    uint8_t who = 0u;

    if (!prv_icm_select_bank(0u)) {
        return false;
    }
    if (!prv_icm_spi_read(0x00u, &who, 1u)) {
        return false;
    }

    return (who == 0xEAu);
}

static bool prv_icm_read_accel(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t b[6];

    if (!prv_icm_select_bank(0u)) {
        return false;
    }
    if (!prv_icm_spi_read(0x2Du, b, 6u)) {
        return false;
    }

    *x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    *y = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    *z = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
    return true;
}
#endif /* ICM20948_CS_Pin */

/* -------------------------------------------------------------------------- */
/* LTC2450 (SPI) 외부 ADC                                                     */
/* -------------------------------------------------------------------------- */
#if defined(ADC_CS_Pin)
static void prv_ltc_cs(bool on) /* on=true: CS low */
{
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port,
                      ADC_CS_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool prv_ltc_read_u16(uint16_t* out)
{
    uint8_t rx[2] = {0u, 0u};
    uint8_t tx[2] = {0u, 0u};

    prv_ltc_cs(true);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 50u);
    prv_ltc_cs(false);

    if (st != HAL_OK) {
        return false;
    }

    *out = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return true;
}

static uint16_t prv_ltc_read_avg(void)
{
    uint16_t s[UI_NODE_LTC_SAMPLE_COUNT];

    for (uint32_t i = 0u; i < UI_NODE_LTC_SAMPLE_COUNT; i++) {
        if (!prv_ltc_read_u16(&s[i])) {
            return 0xFFFFu;
        }
        HAL_Delay(40u);
    }

    return prv_trimmed_mean_u16(s,
                                UI_NODE_LTC_SAMPLE_COUNT,
                                UI_NODE_LTC_TRIM_COUNT);
}
#endif /* ADC_CS_Pin */

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void ND_Sensors_Init(void)
{
    /* ADC_EN 등 초기 상태는 main.c에서 설정됨. */
}

bool ND_Sensors_MeasureAll(ND_SensorResult_t* out, uint8_t sensor_en_mask)
{
    int16_t temp_x10 = (int16_t)0xFFFF;
    int8_t temp_c = UI_NODE_TEMP_INVALID_C;
    uint16_t temp_vdd_mv = 0u;

    if (out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    sensor_en_mask &= UI_SENSOR_EN_ALL;

    /* Stop wake 이후에는 ADC/SPI가 DeInit 상태일 수 있음 -> 필요한 것만 Init */
    prv_ensure_adc_init();
#if defined(ICM20948_CS_Pin) || defined(ADC_CS_Pin)
    prv_ensure_spi_init();
#endif

    /* 기본 invalid */
    out->batt_lvl = UI_NODE_BATT_LVL_INVALID;
    out->temp_c = UI_NODE_TEMP_INVALID_C;
    out->x = (int16_t)0xFFFFu;
    out->y = (int16_t)0xFFFFu;
    out->z = (int16_t)0xFFFFu;
    out->adc = 0;
    out->pulse_cnt = 0;
    if ((sensor_en_mask & UI_SENSOR_EN_PULSE) != 0u) {
        out->pulse_cnt = UI_GPIO_GetPulseCount();
    }

    /*
     * 1) MCU 내부 온도는 내부 ADC로 먼저 측정한다.
     *    batt level은 아래에서 BATT_LVL GPIO로 별도 판정한다.
     */
    if (prv_read_temp_x10_internal(&temp_x10, &temp_vdd_mv) &&
        prv_temp_x10_to_temp_c_checked(temp_x10, &temp_c)) {
        temp_c = prv_apply_temp_offset_c(temp_c);
        out->temp_c = temp_c;
        s_last_valid_temp_c = temp_c;
    } else if (s_last_valid_temp_c != UI_NODE_TEMP_INVALID_C) {
        out->temp_c = s_last_valid_temp_c;
    }

    {
        uint8_t batt_lvl = UI_NODE_BATT_LVL_INVALID;

        (void)temp_vdd_mv; /* batt level은 VDD threshold가 아니라 GPIO 입력으로 판단 */

        if (prv_read_batt_lvl_gpio(&batt_lvl)) {
            out->batt_lvl = batt_lvl;
            s_last_valid_batt_lvl = batt_lvl;
        } else if (s_last_valid_batt_lvl != UI_NODE_BATT_LVL_INVALID) {
            out->batt_lvl = s_last_valid_batt_lvl;
        }
    }

    /* 2) ICM20948 */
#if defined(ICM20948_CS_Pin)
    if (((sensor_en_mask & UI_SENSOR_EN_ICM20948) != 0u) &&
        prv_icm_wakeup() && prv_icm_check_whoami()) {
        int16_t xs[UI_NODE_ICM_SAMPLE_COUNT];
        int16_t ys[UI_NODE_ICM_SAMPLE_COUNT];
        int16_t zs[UI_NODE_ICM_SAMPLE_COUNT];
        bool icm_samples_ok = true;

        for (uint32_t i = 0u; i < UI_NODE_ICM_SAMPLE_COUNT; i++) {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;

            if (!prv_icm_read_accel(&x, &y, &z)) {
                icm_samples_ok = false;
                break;
            }

            xs[i] = x;
            ys[i] = y;
            zs[i] = z;
            HAL_Delay(2u);
        }

        if (icm_samples_ok) {
            out->x = prv_trimmed_mean_i16(xs, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
            out->y = prv_trimmed_mean_i16(ys, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
            out->z = prv_trimmed_mean_i16(zs, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
        } else {
            out->x = (int16_t)0xFFFFu;
            out->y = (int16_t)0xFFFFu;
            out->z = (int16_t)0xFFFFu;
        }
        (void)prv_icm_sleep();
    }
#endif

    /* 3) LTC2450 (ADC_EN is LTC2450 power only) */
#if defined(ADC_CS_Pin)
    if ((sensor_en_mask & UI_SENSOR_EN_ADC) != 0u) {
        prv_set_adc_en(true);
        HAL_Delay(10u);
        out->adc = prv_ltc_read_avg();
    }
#endif

    prv_set_adc_en(false);

    /* 최소 전류: 측정 종료 후 ADC/SPI도 즉시 정리(필요 시 다시 Ensure로 Init) */
#if defined(HAL_ADC_MODULE_ENABLED)
    (void)HAL_ADC_DeInit(&hadc);
#endif
#if defined(HAL_SPI_MODULE_ENABLED)
    (void)HAL_SPI_DeInit(&hspi1);
#endif

    return true;
}
