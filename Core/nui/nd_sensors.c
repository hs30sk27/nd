#include "nd_sensors.h"
#include "ui_conf.h"
#include "ui_gpio.h"
#include "main.h"
#include "stm32wlxx_hal.h"

#include <string.h>

extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;

extern void MX_ADC_Init(void);
extern void MX_SPI1_Init(void);

#ifndef UI_ADC_INTERNAL_SAMPLINGTIME
#if defined(ADC_SAMPLETIME_640CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_640CYCLES_5)
#elif defined(ADC_SAMPLETIME_247CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_247CYCLES_5)
#elif defined(ADC_SAMPLETIME_160CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_160CYCLES_5)
#elif defined(ADC_SAMPLETIME_92CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_92CYCLES_5)
#elif defined(ADC_SAMPLETIME_79CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_79CYCLES_5)
#elif defined(ADC_SAMPLETIME_47CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_47CYCLES_5)
#elif defined(ADC_SAMPLETIME_39CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_39CYCLES_5)
#elif defined(ADC_SAMPLETIME_24CYCLES_5)
#define UI_ADC_INTERNAL_SAMPLINGTIME (ADC_SAMPLETIME_24CYCLES_5)
#else
#define UI_ADC_INTERNAL_SAMPLINGTIME (UI_ADC_SAMPLINGTIME)
#endif
#endif

#ifndef UI_NODE_INTERNAL_SETTLE_DELAY_MS
#define UI_NODE_INTERNAL_SETTLE_DELAY_MS (2u)
#endif

#ifndef UI_NODE_VREF_WARMUP_COUNT
#define UI_NODE_VREF_WARMUP_COUNT (2u)
#endif

#ifndef UI_NODE_VREF_SAMPLE_COUNT
#define UI_NODE_VREF_SAMPLE_COUNT (8u)
#endif

#ifndef UI_NODE_VREF_TRIM_COUNT
#define UI_NODE_VREF_TRIM_COUNT (2u)
#endif

#ifndef UI_NODE_ADC_POWER_SETTLE_MS
#define UI_NODE_ADC_POWER_SETTLE_MS (120u)
#endif

#ifndef UI_NODE_LTC_WARMUP_DISCARD_COUNT
#define UI_NODE_LTC_WARMUP_DISCARD_COUNT (6u)
#endif

#ifndef UI_NODE_TEMP_SUSPECT_LOW_C
#define UI_NODE_TEMP_SUSPECT_LOW_C ((int8_t)-40)
#endif

#ifndef UI_NODE_TEMP_SUSPECT_HIGH_C
#define UI_NODE_TEMP_SUSPECT_HIGH_C ((int8_t)85)
#endif

#ifndef UI_NODE_TEMP_GW_STYLE_RETRY_COUNT
#define UI_NODE_TEMP_GW_STYLE_RETRY_COUNT (3u)
#endif

#ifndef UI_NODE_TEMP_GW_STYLE_RETRY_DELAY_MS
#define UI_NODE_TEMP_GW_STYLE_RETRY_DELAY_MS (5u)
#endif

static void prv_sort_u16(uint16_t *a, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++) {
        uint16_t key = a[i];
        int j = (int)i - 1;
        while ((j >= 0) && (a[j] > key)) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

static void prv_sort_i16(int16_t *a, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++) {
        int16_t key = a[i];
        int j = (int)i - 1;
        while ((j >= 0) && (a[j] > key)) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

static uint16_t prv_trimmed_mean_u16(uint16_t *a, uint16_t n, uint16_t trim_each_side)
{
    uint16_t start;
    uint16_t end;
    uint32_t sum = 0u;
    uint32_t cnt = 0u;

    if (n == 0u) {
        return 0xFFFFu;
    }

    prv_sort_u16(a, n);
    start = trim_each_side;
    end = (uint16_t)(n - trim_each_side);
    if (end <= start) {
        return a[n / 2u];
    }

    for (uint16_t i = start; i < end; i++) {
        sum += a[i];
        cnt++;
    }

    return (cnt == 0u) ? a[n / 2u] : (uint16_t)(sum / cnt);
}

static int16_t prv_trimmed_mean_i16(int16_t *a, uint16_t n, uint16_t trim_each_side)
{
    uint16_t start;
    uint16_t end;
    int32_t sum = 0;
    uint32_t cnt = 0u;

    if (n == 0u) {
        return (int16_t)0xFFFFu;
    }

    prv_sort_i16(a, n);
    start = trim_each_side;
    end = (uint16_t)(n - trim_each_side);
    if (end <= start) {
        return a[n / 2u];
    }

    for (uint16_t i = start; i < end; i++) {
        sum += a[i];
        cnt++;
    }

    return (cnt == 0u) ? a[n / 2u] : (int16_t)(sum / (int32_t)cnt);
}

static void prv_set_adc_en(bool on)
{
#if defined(ADC_EN_Pin)
    HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

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

static bool prv_adc_read(uint32_t channel, uint32_t sampling_time, uint16_t *out_raw)
{
#if defined(HAL_ADC_MODULE_ENABLED)
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampling_time;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        return false;
    }
    if (HAL_ADC_Start(&hadc) != HAL_OK) {
        return false;
    }
    if (HAL_ADC_PollForConversion(&hadc, 50) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc);
        return false;
    }
    *out_raw = (uint16_t)HAL_ADC_GetValue(&hadc);
    (void)HAL_ADC_Stop(&hadc);
    return true;
#else
    (void)channel;
    (void)sampling_time;
    (void)out_raw;
    return false;
#endif
}

static bool prv_adc_read_quick(uint32_t channel, uint16_t *out_raw)
{
    return prv_adc_read(channel, UI_ADC_SAMPLINGTIME, out_raw);
}

static uint16_t prv_read_vdd_mv_quick(void)
{
#if defined(ADC_CHANNEL_VREFINT)
    uint16_t raw = 0u;
    if (!prv_adc_read_quick(ADC_CHANNEL_VREFINT, &raw)) {
        return 0u;
    }
#if defined(__HAL_ADC_CALC_VREFANALOG_VOLTAGE)
    return (uint16_t)__HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_12B);
#else
    (void)raw;
    return 0u;
#endif
#else
    return 0u;
#endif
}

static int16_t prv_read_temp_x10_quick(uint16_t vdd_mv)
{
#if defined(ADC_CHANNEL_TEMPSENSOR)
    uint16_t samples[UI_NODE_TEMP_SAMPLE_COUNT];
    uint16_t raw_mid;

    if (vdd_mv == 0u) {
        return (int16_t)0xFFFFu;
    }

    for (uint32_t i = 0u; i < UI_NODE_TEMP_SAMPLE_COUNT; i++) {
        if (!prv_adc_read_quick(ADC_CHANNEL_TEMPSENSOR, &samples[i])) {
            return (int16_t)0xFFFFu;
        }
        HAL_Delay(2u);
    }

    raw_mid = prv_trimmed_mean_u16(samples, UI_NODE_TEMP_SAMPLE_COUNT, UI_NODE_TEMP_TRIM_COUNT);
#if defined(__HAL_ADC_CALC_TEMPERATURE)
    return (int16_t)(__HAL_ADC_CALC_TEMPERATURE(vdd_mv, raw_mid, ADC_RESOLUTION_12B) * 10);
#else
    (void)raw_mid;
    return (int16_t)0xFFFFu;
#endif
#else
    (void)vdd_mv;
    return (int16_t)0xFFFFu;
#endif
}

static uint16_t prv_read_vdd_mv(void)
{
#if defined(ADC_CHANNEL_VREFINT)
    uint16_t samples[UI_NODE_VREF_SAMPLE_COUNT];

    for (uint32_t i = 0u; i < UI_NODE_VREF_WARMUP_COUNT; i++) {
        uint16_t raw_dummy = 0u;
        if (!prv_adc_read(ADC_CHANNEL_VREFINT, UI_ADC_INTERNAL_SAMPLINGTIME, &raw_dummy)) {
            return 0u;
        }
        HAL_Delay(UI_NODE_INTERNAL_SETTLE_DELAY_MS);
    }

    for (uint32_t i = 0u; i < UI_NODE_VREF_SAMPLE_COUNT; i++) {
        if (!prv_adc_read(ADC_CHANNEL_VREFINT, UI_ADC_INTERNAL_SAMPLINGTIME, &samples[i])) {
            return 0u;
        }
        HAL_Delay(UI_NODE_INTERNAL_SETTLE_DELAY_MS);
    }

    uint16_t raw = prv_trimmed_mean_u16(samples, UI_NODE_VREF_SAMPLE_COUNT, UI_NODE_VREF_TRIM_COUNT);
#if defined(__HAL_ADC_CALC_VREFANALOG_VOLTAGE)
    return (uint16_t)__HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_12B);
#else
    (void)raw;
    return 0u;
#endif
#else
    return 0u;
#endif
}

static int16_t prv_read_temp_x10(void)
{
#if defined(ADC_CHANNEL_TEMPSENSOR)
    uint16_t samples[UI_NODE_TEMP_SAMPLE_COUNT];
    uint16_t vdd_mv;
    uint16_t raw_mid;

    for (uint32_t i = 0u; i < UI_NODE_TEMP_WARMUP_COUNT; i++) {
        uint16_t raw_dummy = 0u;
        if (!prv_adc_read(ADC_CHANNEL_TEMPSENSOR, UI_ADC_INTERNAL_SAMPLINGTIME, &raw_dummy)) {
            return (int16_t)0xFFFFu;
        }
        HAL_Delay(UI_NODE_TEMP_WARMUP_DELAY_MS);
    }

    for (uint32_t i = 0u; i < UI_NODE_TEMP_SAMPLE_COUNT; i++) {
        if (!prv_adc_read(ADC_CHANNEL_TEMPSENSOR, UI_ADC_INTERNAL_SAMPLINGTIME, &samples[i])) {
            return (int16_t)0xFFFFu;
        }
        HAL_Delay(UI_NODE_INTERNAL_SETTLE_DELAY_MS);
    }

    raw_mid = prv_trimmed_mean_u16(samples, UI_NODE_TEMP_SAMPLE_COUNT, UI_NODE_TEMP_TRIM_COUNT);
    vdd_mv = prv_read_vdd_mv();
    if (vdd_mv == 0u) {
        return (int16_t)0xFFFFu;
    }

#if defined(__HAL_ADC_CALC_TEMPERATURE)
    return (int16_t)(__HAL_ADC_CALC_TEMPERATURE(vdd_mv, raw_mid, ADC_RESOLUTION_12B) * 10);
#else
    (void)raw_mid;
    return (int16_t)0xFFFFu;
#endif
#else
    return (int16_t)0xFFFFu;
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

static int8_t prv_temp_x10_to_temp_c(int16_t temp_x10)
{
    int16_t temp_c;

    if ((uint16_t)temp_x10 == 0xFFFFu) {
        return UI_NODE_TEMP_INVALID_C;
    }

    if (temp_x10 >= 0) {
        temp_c = (int16_t)((temp_x10 + 5) / 10);
    } else {
        temp_c = (int16_t)((temp_x10 - 5) / 10);
    }

    return prv_clamp_temp_c_i16(temp_c);
}

static int8_t prv_apply_temp_offset_c(int8_t temp_c)
{
    if (temp_c == UI_NODE_TEMP_INVALID_C) {
        return UI_NODE_TEMP_INVALID_C;
    }
    return prv_clamp_temp_c_i16((int16_t)temp_c + (int16_t)UI_NODE_TEMP_OFFSET_C);
}

static uint16_t prv_read_vdd_x10(void);

static bool prv_temp_c_looks_suspicious(int8_t temp_c)
{
    if (temp_c == UI_NODE_TEMP_INVALID_C) {
        return true;
    }
    if (temp_c <= UI_NODE_TEMP_SUSPECT_LOW_C) {
        return true;
    }
    if (temp_c >= UI_NODE_TEMP_SUSPECT_HIGH_C) {
        return true;
    }
    return false;
}

static bool prv_read_vdd_mv_gw_style(uint16_t *out_vdd_mv)
{
#if defined(ADC_CHANNEL_VREFINT) && defined(__HAL_ADC_CALC_VREFANALOG_VOLTAGE)
    uint16_t raw = 0u;

    if ((out_vdd_mv == NULL) || !prv_adc_read(ADC_CHANNEL_VREFINT, UI_ADC_SAMPLINGTIME, &raw)) {
        return false;
    }

    *out_vdd_mv = (uint16_t)__HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_12B);
    return (*out_vdd_mv != 0u);
#else
    (void)out_vdd_mv;
    return false;
#endif
}

static bool prv_read_temp_x10_gw_style(uint16_t vdd_mv, int16_t *out_temp_x10)
{
#if defined(ADC_CHANNEL_TEMPSENSOR) && defined(__HAL_ADC_CALC_TEMPERATURE)
    uint16_t samples[10];
    uint16_t raw_mid;

    if ((out_temp_x10 == NULL) || (vdd_mv == 0u)) {
        return false;
    }

    for (uint32_t i = 0u; i < 10u; i++) {
        if (!prv_adc_read(ADC_CHANNEL_TEMPSENSOR, UI_ADC_SAMPLINGTIME, &samples[i])) {
            return false;
        }
        HAL_Delay(2u);
    }

    raw_mid = prv_trimmed_mean_u16(samples, 10u, 2u);
    *out_temp_x10 = (int16_t)(__HAL_ADC_CALC_TEMPERATURE(vdd_mv, raw_mid, ADC_RESOLUTION_12B) * 10);
    return true;
#else
    (void)vdd_mv;
    (void)out_temp_x10;
    return false;
#endif
}

static bool prv_measure_internal_primary(uint16_t *out_vdd_x10, int8_t *out_temp_c)
{
    uint16_t vdd_mv = 0u;
    int16_t temp_x10 = (int16_t)0xFFFFu;
    int8_t temp_c = UI_NODE_TEMP_INVALID_C;
    bool have_value = false;

    if ((out_vdd_x10 == NULL) || (out_temp_c == NULL)) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < UI_NODE_TEMP_GW_STYLE_RETRY_COUNT; attempt++) {
        if (!prv_read_vdd_mv_gw_style(&vdd_mv) || (vdd_mv == 0u)) {
            HAL_Delay(UI_NODE_TEMP_GW_STYLE_RETRY_DELAY_MS);
            continue;
        }

        *out_vdd_x10 = (uint16_t)((vdd_mv + 50u) / 100u);
        if (!prv_read_temp_x10_gw_style(vdd_mv, &temp_x10)) {
            HAL_Delay(UI_NODE_TEMP_GW_STYLE_RETRY_DELAY_MS);
            continue;
        }

        temp_c = prv_apply_temp_offset_c(prv_temp_x10_to_temp_c(temp_x10));
        *out_temp_c = temp_c;
        have_value = (temp_c != UI_NODE_TEMP_INVALID_C);
        if (have_value && !prv_temp_c_looks_suspicious(temp_c)) {
            return true;
        }

        HAL_Delay(UI_NODE_TEMP_GW_STYLE_RETRY_DELAY_MS);
    }

    return have_value;
}

static bool prv_measure_internal_fallback(uint16_t *out_vdd_x10, int8_t *out_temp_c)
{
    uint16_t vdd_x10;
    int8_t temp_c;

    if ((out_vdd_x10 == NULL) || (out_temp_c == NULL)) {
        return false;
    }

    vdd_x10 = prv_read_vdd_x10();
    temp_c = prv_apply_temp_offset_c(prv_temp_x10_to_temp_c(prv_read_temp_x10()));

    if (vdd_x10 == 0xFFFFu) {
        return false;
    }

    *out_vdd_x10 = vdd_x10;
    *out_temp_c = temp_c;
    return (temp_c != UI_NODE_TEMP_INVALID_C);
}

static uint16_t prv_read_vdd_x10(void)
{
    uint16_t vdd_mv = prv_read_vdd_mv();
    if (vdd_mv == 0u) {
        return 0xFFFFu;
    }
    return (uint16_t)((vdd_mv + 50u) / 100u);
}

#if defined(ICM20948_CS_Pin)
static void prv_icm_cs(bool on)
{
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool prv_icm_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
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

static bool prv_icm_read_accel(int16_t *x, int16_t *y, int16_t *z)
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
#endif

#if defined(ADC_CS_Pin)
static void prv_ltc_cs(bool on)
{
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool prv_ltc_read_u16(uint16_t *out)
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
    uint16_t dummy = 0u;

    for (uint32_t i = 0u; i < UI_NODE_LTC_WARMUP_DISCARD_COUNT; i++) {
        if (!prv_ltc_read_u16(&dummy)) {
            return 0xFFFFu;
        }
        HAL_Delay(40u);
    }

    for (uint32_t i = 0u; i < UI_NODE_LTC_SAMPLE_COUNT; i++) {
        if (!prv_ltc_read_u16(&s[i])) {
            return 0xFFFFu;
        }
        HAL_Delay(40u);
    }
    return prv_trimmed_mean_u16(s, UI_NODE_LTC_SAMPLE_COUNT, UI_NODE_LTC_TRIM_COUNT);
}
#endif

void ND_Sensors_Init(void)
{
}

bool ND_Sensors_MeasureAll(ND_SensorResult_t *out)
{
    uint16_t vdd_x10 = 0xFFFFu;
    int8_t temp_c = UI_NODE_TEMP_INVALID_C;
    bool internal_ok = false;

    if (out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    prv_ensure_adc_init();
#if defined(ICM20948_CS_Pin) || defined(ADC_CS_Pin)
    prv_ensure_spi_init();
#endif

    out->batt_lvl = UI_NODE_BATT_LVL_LOW;
    out->temp_c = UI_NODE_TEMP_INVALID_C;
    out->x = (int16_t)0xFFFFu;
    out->y = (int16_t)0xFFFFu;
    out->z = (int16_t)0xFFFFu;
    out->adc = 0xFFFFu;
    out->pulse_cnt = UI_GPIO_GetPulseCount();

    /* 내부 ADC는 GW와 같은 조건으로 ADC_EN ON 상태에서 먼저 읽는다. */
    prv_set_adc_en(true);
    HAL_Delay(UI_NODE_INTERNAL_SETTLE_DELAY_MS);

    internal_ok = prv_measure_internal_primary(&vdd_x10, &temp_c);
    if ((!internal_ok) || prv_temp_c_looks_suspicious(temp_c)) {
        uint16_t vdd_x10_fb = 0xFFFFu;
        int8_t temp_c_fb = UI_NODE_TEMP_INVALID_C;
        if (prv_measure_internal_fallback(&vdd_x10_fb, &temp_c_fb)) {
            vdd_x10 = vdd_x10_fb;
            temp_c = temp_c_fb;
            internal_ok = !prv_temp_c_looks_suspicious(temp_c_fb);
        }
    }

    if ((vdd_x10 != 0xFFFFu) && (vdd_x10 >= UI_NODE_BATT_LOW_THRESHOLD_X10)) {
        out->batt_lvl = UI_NODE_BATT_LVL_NORMAL;
    }
    out->temp_c = temp_c;

#if defined(ICM20948_CS_Pin) || defined(ADC_CS_Pin)
    /* 외부 ADC/IMU는 같은 ADC_EN 상태를 유지한 채 추가 settle만 준다. */
    HAL_Delay(UI_NODE_ADC_POWER_SETTLE_MS);
#endif

#if defined(ADC_CS_Pin)
    /* 아날로그 샘플은 ICM SPI 트래픽 전에 먼저 읽어서 GW 쪽 값과 시간차를 줄인다. */
    out->adc = prv_ltc_read_avg();
#endif

#if defined(ICM20948_CS_Pin)
    if (prv_icm_wakeup() && prv_icm_check_whoami()) {
        int16_t xs[UI_NODE_ICM_SAMPLE_COUNT];
        int16_t ys[UI_NODE_ICM_SAMPLE_COUNT];
        int16_t zs[UI_NODE_ICM_SAMPLE_COUNT];
        for (uint32_t i = 0u; i < UI_NODE_ICM_SAMPLE_COUNT; i++) {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
            if (!prv_icm_read_accel(&x, &y, &z)) {
                x = (int16_t)0xFFFFu;
                y = (int16_t)0xFFFFu;
                z = (int16_t)0xFFFFu;
            }
            xs[i] = x;
            ys[i] = y;
            zs[i] = z;
            HAL_Delay(2u);
        }
        out->x = prv_trimmed_mean_i16(xs, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
        out->y = prv_trimmed_mean_i16(ys, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
        out->z = prv_trimmed_mean_i16(zs, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
    }
#endif

#if defined(ICM20948_CS_Pin) || defined(ADC_CS_Pin)
    prv_set_adc_en(false);
#endif

#if defined(HAL_ADC_MODULE_ENABLED)
    (void)HAL_ADC_DeInit(&hadc);
#endif
#if defined(HAL_SPI_MODULE_ENABLED)
    (void)HAL_SPI_DeInit(&hspi1);
#endif

    return true;
}

