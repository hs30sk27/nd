#include "nd_sensors.h"
#include "ui_conf.h"
#include "ui_gpio.h"

#include "main.h"
#include "stm32wlxx_hal.h"
#include <string.h>

/* 프로젝트 핸들 */
extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;

/* main.c에 생성된 Init 함수(Stop wake 후 필요할 때만 호출) */
extern void MX_ADC_Init(void);
extern void MX_SPI1_Init(void);

/* -------------------------------------------------------------------------- */
/* 유틸: 정렬 + 트림 평균                                                     */
/* -------------------------------------------------------------------------- */
static void prv_sort_u16(uint16_t* a, uint16_t n)
{
    /* insertion sort (n<=100) */
    for (uint16_t i = 1; i < n; i++)
    {
        uint16_t key = a[i];
        int j = (int)i - 1;
        while (j >= 0 && a[j] > key)
        {
            a[j+1] = a[j];
            j--;
        }
        a[j+1] = key;
    }
}

static void prv_sort_i16(int16_t* a, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++)
    {
        int16_t key = a[i];
        int j = (int)i - 1;
        while (j >= 0 && a[j] > key)
        {
            a[j+1] = a[j];
            j--;
        }
        a[j+1] = key;
    }
}

static uint16_t prv_trimmed_mean_u16(uint16_t* a, uint16_t n, uint16_t trim_each_side)
{
    if (n == 0u) return 0xFFFFu;
    prv_sort_u16(a, n);

    uint16_t start = trim_each_side;
    uint16_t end   = (uint16_t)(n - trim_each_side);
    if (end <= start) return a[n/2u];

    uint32_t sum = 0;
    uint32_t cnt = 0;
    for (uint16_t i = start; i < end; i++)
    {
        sum += a[i];
        cnt++;
    }
    return (uint16_t)(sum / cnt);
}

static int16_t prv_trimmed_mean_i16(int16_t* a, uint16_t n, uint16_t trim_each_side)
{
    if (n == 0u) return (int16_t)0xFFFFu;
    prv_sort_i16(a, n);

    uint16_t start = trim_each_side;
    uint16_t end   = (uint16_t)(n - trim_each_side);
    if (end <= start) return a[n/2u];

    int32_t sum = 0;
    uint32_t cnt = 0;
    for (uint16_t i = start; i < end; i++)
    {
        sum += a[i];
        cnt++;
    }
    return (int16_t)(sum / (int32_t)cnt);
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
/* Init Ensure (Stop wake 이후 필요한 주변장치만 Init) */
/* -------------------------------------------------------------------------- */
static void prv_ensure_adc_init(void)
{
#if defined(HAL_ADC_MODULE_ENABLED)
    if (hadc.State == HAL_ADC_STATE_RESET)
    {
        MX_ADC_Init();
    }
#endif
}

static void prv_ensure_spi_init(void)
{
#if defined(HAL_SPI_MODULE_ENABLED)
    if (hspi1.State == HAL_SPI_STATE_RESET)
    {
        MX_SPI1_Init();
    }
#endif
}

/* -------------------------------------------------------------------------- */
/* 내부 ADC (VREFINT/TEMPSENSOR)                                              */
/* -------------------------------------------------------------------------- */
static bool prv_adc_read(uint32_t channel, uint16_t* out_raw)
{
#if defined(HAL_ADC_MODULE_ENABLED)
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = UI_ADC_SAMPLINGTIME; /* 내부 채널용 sampling time */
    /*
     * NOTE(호환성):
     *  - 일부 STM32 HAL 버전/시리즈에서는 ADC_ChannelConfTypeDef에
     *    SingleDiff/OffsetNumber/Offset 멤버가 없습니다.
     *  - 구조체를 {0}로 초기화하면 대부분의 디폴트(싱글엔디드/오프셋 없음)가 0이므로
     *    호환성을 위해 여기서는 필드를 직접 건드리지 않습니다.
     */

    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) return false;
    if (HAL_ADC_Start(&hadc) != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(&hadc, 50) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc);
        return false;
    }
    uint32_t v = HAL_ADC_GetValue(&hadc);
    (void)HAL_ADC_Stop(&hadc);

    *out_raw = (uint16_t)v;
    return true;
#else
    (void)channel;
    (void)out_raw;
    return false;
#endif
}

static uint16_t prv_read_vdd_mv(void)
{
    /* VREFINT 측정으로 VDD 계산 (HAL macro가 없으면 0) */
#if defined(ADC_CHANNEL_VREFINT)
    uint16_t raw = 0;
    if (!prv_adc_read(ADC_CHANNEL_VREFINT, &raw)) return 0;

#if defined(__HAL_ADC_CALC_VREFANALOG_VOLTAGE)
    /* 이 매크로는 device별로 정의됨 */
    uint32_t vdd_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_12B);
    return (uint16_t)vdd_mv;
#else
    (void)raw;
    return 0;
#endif
#else
    return 0;
#endif
}

static int16_t prv_read_temp_x10(void)
{
#if defined(ADC_CHANNEL_TEMPSENSOR)
    uint16_t samples[UI_NODE_TEMP_SAMPLE_COUNT];

    /*
     * ND는 외부 센서 전원/LoRa 동작 직후에 온도를 읽으면 실제 체감 온도보다 높게
     * 보일 수 있어, 내부 채널 전환 직후의 warm-up 변환을 버리고 본 측정을 시작합니다.
     * 문서 요구인 "10회 측정 + 가운데 6개 평균"은 그대로 유지합니다.
     */
    for (uint32_t i = 0; i < UI_NODE_TEMP_WARMUP_COUNT; i++)
    {
        uint16_t raw_dummy = 0;
        if (!prv_adc_read(ADC_CHANNEL_TEMPSENSOR, &raw_dummy))
        {
            return (int16_t)0xFFFFu;
        }
        HAL_Delay(UI_NODE_TEMP_WARMUP_DELAY_MS);
    }

    for (uint32_t i = 0; i < UI_NODE_TEMP_SAMPLE_COUNT; i++)
    {
        uint16_t raw = 0;
        if (!prv_adc_read(ADC_CHANNEL_TEMPSENSOR, &raw))
        {
            return (int16_t)0xFFFFu;
        }
        samples[i] = raw;
        HAL_Delay(2);
    }

    /* 정렬 후 양끝값을 버리고 중간 샘플 평균 */
    uint16_t raw_mid = prv_trimmed_mean_u16(samples, UI_NODE_TEMP_SAMPLE_COUNT, UI_NODE_TEMP_TRIM_COUNT);

    uint16_t vdd_mv = prv_read_vdd_mv();
    if (vdd_mv == 0) return (int16_t)0xFFFFu;

#if defined(__HAL_ADC_CALC_TEMPERATURE)
    int32_t temp_c = __HAL_ADC_CALC_TEMPERATURE(vdd_mv, raw_mid, ADC_RESOLUTION_12B);
    /* 0.1'C */
    return (int16_t)(temp_c * 10);
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
    if (temp_c < (int16_t)UI_NODE_TEMP_MIN_C)
    {
        temp_c = (int16_t)UI_NODE_TEMP_MIN_C;
    }
    if (temp_c > (int16_t)UI_NODE_TEMP_MAX_C)
    {
        temp_c = (int16_t)UI_NODE_TEMP_MAX_C;
    }
    return (int8_t)temp_c;
}

static int8_t prv_temp_x10_to_temp_c(int16_t temp_x10)
{
    if ((uint16_t)temp_x10 == 0xFFFFu)
    {
        return UI_NODE_TEMP_INVALID_C;
    }

    int16_t temp_c;
    if (temp_x10 >= 0)
    {
        temp_c = (int16_t)((temp_x10 + 5) / 10);
    }
    else
    {
        temp_c = (int16_t)((temp_x10 - 5) / 10);
    }

    return prv_clamp_temp_c_i16(temp_c);
}

static int8_t prv_apply_temp_offset_c(int8_t temp_c)
{
    if (temp_c == UI_NODE_TEMP_INVALID_C)
    {
        return UI_NODE_TEMP_INVALID_C;
    }

    return prv_clamp_temp_c_i16((int16_t)temp_c + (int16_t)UI_NODE_TEMP_OFFSET_C);
}

static uint16_t prv_read_vdd_x10(void)
{
    uint16_t vdd_mv = prv_read_vdd_mv();
    if (vdd_mv == 0u) return 0xFFFFu;

    /* 0.1V = 100mV */
    return (uint16_t)((vdd_mv + 50u) / 100u);
}

/* -------------------------------------------------------------------------- */
/* ICM20948 (SPI)                                                             */
/* -------------------------------------------------------------------------- */
#if defined(ICM20948_CS_Pin)

static void prv_icm_cs(bool on) /* on=true: CS low */
{
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool prv_icm_spi_read(uint8_t reg, uint8_t* buf, uint16_t len)
{
    /* SPI read: reg | 0x80 */
    uint8_t addr = (uint8_t)(reg | 0x80u);
    prv_icm_cs(true);
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, 50) != HAL_OK)
    {
        prv_icm_cs(false);
        return false;
    }
    if (HAL_SPI_Receive(&hspi1, buf, len, 50) != HAL_OK)
    {
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
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, 2, 50);
    prv_icm_cs(false);
    return (st == HAL_OK);
}

static bool prv_icm_select_bank(uint8_t bank)
{
    /* REG_BANK_SEL = 0x7F */
    return prv_icm_spi_write(0x7Fu, (uint8_t)(bank << 4));
}

static bool prv_icm_wakeup(void)
{
    /* bank 0 */
    if (!prv_icm_select_bank(0)) return false;

    /* PWR_MGMT_1 = 0x06, set to 0x01 (auto clk, sleep=0) */
    if (!prv_icm_spi_write(0x06u, 0x01u)) return false;

    HAL_Delay(10);
    return true;
}

static bool prv_icm_check_whoami(void)
{
    if (!prv_icm_select_bank(0)) return false;

    uint8_t who = 0;
    if (!prv_icm_spi_read(0x00u, &who, 1u)) return false;

    /* ICM-20948 WHO_AM_I = 0xEA */
    return (who == 0xEAu);
}

static bool prv_icm_read_accel(int16_t* x, int16_t* y, int16_t* z)
{
    if (!prv_icm_select_bank(0)) return false;

    uint8_t b[6];
    /* ACCEL_XOUT_H = 0x2D */
    if (!prv_icm_spi_read(0x2Du, b, 6u)) return false;

    *x = (int16_t)((int16_t)((uint16_t)b[0] << 8) | b[1]);
    *y = (int16_t)((int16_t)((uint16_t)b[2] << 8) | b[3]);
    *z = (int16_t)((int16_t)((uint16_t)b[4] << 8) | b[5]);
    return true;
}

#endif /* ICM20948_CS_Pin */

/* -------------------------------------------------------------------------- */
/* LTC2450 (SPI) 외부 ADC                                                     */
/* -------------------------------------------------------------------------- */
#if defined(ADC_CS_Pin)

static void prv_ltc_cs(bool on) /* on=true: CS low */
{
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool prv_ltc_read_u16(uint16_t* out)
{
    uint8_t rx[2] = {0,0};
    uint8_t tx[2] = {0,0};

    prv_ltc_cs(true);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 50);
    prv_ltc_cs(false);

    if (st != HAL_OK) return false;

    *out = (uint16_t)((uint16_t)rx[0] << 8) | rx[1];
    return true;
}

static uint16_t prv_ltc_read_avg(void)
{
    /* 샘플을 모두 모은 뒤 sort -> 양끝 trim -> 중간 평균 */
    uint16_t s[UI_NODE_LTC_SAMPLE_COUNT];
    for (uint32_t i = 0; i < UI_NODE_LTC_SAMPLE_COUNT; i++)
    {
        if (!prv_ltc_read_u16(&s[i]))
        {
            return 0xFFFFu;
        }
        /* 변환 시간 고려(데이터시트 기준 수십 ms) */
        HAL_Delay(40);
    }

    return prv_trimmed_mean_u16(s, UI_NODE_LTC_SAMPLE_COUNT, UI_NODE_LTC_TRIM_COUNT);
}

#endif /* ADC_CS_Pin */

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void ND_Sensors_Init(void)
{
    /* ADC_EN 등 초기 상태는 main.c에서 설정됨. 여기서는 별도 없음. */
}

bool ND_Sensors_MeasureAll(ND_SensorResult_t* out)
{
    if (out == NULL) return false;

    memset(out, 0, sizeof(*out));

    /* Stop wake 이후에는 ADC/SPI가 DeInit 상태일 수 있음 -> 필요한 것만 Init */
    prv_ensure_adc_init();
#if defined(ICM20948_CS_Pin) || defined(ADC_CS_Pin)
    prv_ensure_spi_init();
#endif

    /* 기본 invalid */
    out->batt_lvl  = UI_NODE_BATT_LVL_LOW;
    out->temp_c    = UI_NODE_TEMP_INVALID_C;
    out->x         = (int16_t)0xFFFFu;
    out->y         = (int16_t)0xFFFFu;
    out->z         = (int16_t)0xFFFFu;
    out->adc       = 0xFFFFu;
    out->pulse_cnt = UI_GPIO_GetPulseCount();

    /*
     * 1) 배터리 레벨 / 내부 온도
     *    - GW는 외부 센서 전원과 무관하게 내부 ADC 값만 읽습니다.
     *    - ND도 동일하게 먼저 내부 채널을 읽고, 이후에만 ADC_EN을 올려 외부 센서를 측정합니다.
     *      이렇게 하면 외부 센서 전원 ON 직후의 국부적인 발열/노이즈 영향을 줄일 수 있습니다.
     */
    {
        uint16_t vdd_x10 = prv_read_vdd_x10();
        if ((vdd_x10 != 0xFFFFu) && (vdd_x10 >= UI_NODE_BATT_LOW_THRESHOLD_X10))
        {
            out->batt_lvl = UI_NODE_BATT_LVL_NORMAL;
        }
    }

    out->temp_c = prv_apply_temp_offset_c(prv_temp_x10_to_temp_c(prv_read_temp_x10()));

    /* 2) ADC_EN: ICM/LTC 전원 */
    prv_set_adc_en(true);
    HAL_Delay(10);

    /* 3) ICM20948 */
#if defined(ICM20948_CS_Pin)
    if (prv_icm_wakeup() && prv_icm_check_whoami())
    {
        int16_t xs[UI_NODE_ICM_SAMPLE_COUNT], ys[UI_NODE_ICM_SAMPLE_COUNT], zs[UI_NODE_ICM_SAMPLE_COUNT];
        for (uint32_t i = 0; i < UI_NODE_ICM_SAMPLE_COUNT; i++)
        {
            int16_t x=0,y=0,z=0;
            if (!prv_icm_read_accel(&x,&y,&z))
            {
                x = (int16_t)0xFFFFu;
                y = (int16_t)0xFFFFu;
                z = (int16_t)0xFFFFu;
            }
            xs[i] = x;
            ys[i] = y;
            zs[i] = z;
            HAL_Delay(2);
        }

        /* 정렬 후 양끝값을 버리고 중간 샘플 평균 */
        out->x = prv_trimmed_mean_i16(xs, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
        out->y = prv_trimmed_mean_i16(ys, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
        out->z = prv_trimmed_mean_i16(zs, UI_NODE_ICM_SAMPLE_COUNT, UI_NODE_ICM_TRIM_COUNT);
    }
#endif

    /* 4) LTC2450 */
#if defined(ADC_CS_Pin)
    out->adc = prv_ltc_read_avg();
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
