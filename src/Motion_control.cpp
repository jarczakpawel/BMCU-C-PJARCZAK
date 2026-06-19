#include "Motion_control.h"
#include "ams.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "_bus_hardware.h"
#include "many_soft_AS5600.h"
#include "app_api.h"
#include "hal/time_hw.h"

#if BMCU_ONLINE_LED_FILAMENT_RGB
// RGB on, set channel system led color as white
constexpr uint8_t STU_OK_R = 0x38u;
constexpr uint8_t STU_OK_G = 0x35u;
constexpr uint8_t STU_OK_B = 0x32u;
#else
// RGB off, disable channel system led
constexpr uint8_t STU_OK_R = 0x00u;
constexpr uint8_t STU_OK_G = 0x00u;
constexpr uint8_t STU_OK_B = 0x00u;
#endif

static inline float absf(float x) { return (x < 0.0f) ? -x : x; }
static inline float clampf(float x, float a, float b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

static inline uint8_t dm_key_v_to_centi_ceil(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

static inline float dm_key_centi_to_v(uint8_t cv)
{
    return 0.01f * (float)cv;
}

static uint64_t g_time_last_ticks64 = 0ull;
static uint32_t g_time_rem_ticks32  = 0u;
static uint64_t g_time_ms64         = 0ull;
static uint32_t g_time_tpm_last     = 0u;
static uint8_t  g_time_inited       = 0u;

static inline __attribute__((always_inline)) uint64_t time_ms_fast_from_ticks64(uint64_t now_ticks)
{
    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    if (!g_time_inited || (tpm != g_time_tpm_last))
    {
        g_time_inited = 1u;
        g_time_tpm_last = tpm;
        g_time_last_ticks64 = now_ticks;

        g_time_ms64 = now_ticks / (uint64_t)tpm;
        g_time_rem_ticks32 = (uint32_t)(now_ticks - g_time_ms64 * (uint64_t)tpm);
        return g_time_ms64;
    }

    const uint64_t dt64 = now_ticks - g_time_last_ticks64;
    g_time_last_ticks64 = now_ticks;

    if (__builtin_expect(dt64 > 0xFFFFFFFFull, 0))
    {
        g_time_ms64 = now_ticks / (uint64_t)tpm;
        g_time_rem_ticks32 = (uint32_t)(now_ticks - g_time_ms64 * (uint64_t)tpm);
        return g_time_ms64;
    }

    const uint32_t dt  = (uint32_t)dt64;
    const uint32_t rem = g_time_rem_ticks32;

    if (__builtin_expect(dt > (0xFFFFFFFFu - rem), 0))
    {
        g_time_ms64 = now_ticks / (uint64_t)tpm;
        g_time_rem_ticks32 = (uint32_t)(now_ticks - g_time_ms64 * (uint64_t)tpm);
        return g_time_ms64;
    }

    const uint32_t acc = dt + rem;

    if (tpm <= 1u)
    {
        g_time_ms64 += (uint64_t)acc;
        g_time_rem_ticks32 = 0u;
        return g_time_ms64;
    }

    const uint32_t inc = acc / tpm;
    g_time_rem_ticks32 = acc - inc * tpm;

    g_time_ms64 += (uint64_t)inc;
    return g_time_ms64;
}

static inline __attribute__((always_inline)) uint64_t time_ms_fast(void)
{
    return time_ms_fast_from_ticks64(time_ticks64());
}

static inline float retract_mag_from_err(float err, float mag_max)
{
    constexpr float e0 = 0.10f;
    constexpr float e1 = 0.35f;
    constexpr float e2 = 2.35f;

    if (err <= e0) return 0.0f;

    float mag;
    if (err < e1)
    {
        float t = (err - e0) / (e1 - e0);
        t = clampf(t, 0.0f, 1.0f);
        mag = 450.0f + 100.0f * t;
    }
    else
    {
        float t = (err - e1) / (e2 - e1);
        t = clampf(t, 0.0f, 1.0f);
        mag = 550.0f + 300.0f * t;
    }

    if (mag > mag_max) mag = mag_max;
    return mag;
}


static inline uint8_t hyst_u8(uint8_t active, float v, float start, float stop)
{
    if (active) { if (v <= stop)  active = 0; }
    else        { if (v >= start) active = 1; }
    return active;
}


static constexpr uint8_t  kChCount = 4;
static constexpr int      PWM_lim  = 1000;
static constexpr float    kAS5600_PI = 3.14159265358979323846f;

// stała do przeliczenia AS5600 - liczona raz
static constexpr float kAS5600_MM_PER_CNT = -(kAS5600_PI * 7.5f) / 4096.0f;

// ===== AS5600 =====
AS5600_soft_IIC_many MC_AS5600;
static GPIO_TypeDef* const AS5600_SCL_PORT[4] = { GPIOB, GPIOB, GPIOB, GPIOB };
static const uint16_t      AS5600_SCL_PIN [4] = { GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13, GPIO_Pin_12 };
static GPIO_TypeDef* const AS5600_SDA_PORT[4] = { GPIOD, GPIOC, GPIOC, GPIOC };
static const uint16_t      AS5600_SDA_PIN [4] = { GPIO_Pin_0, GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13 };

float speed_as5600[4] = {0, 0, 0, 0};
// ===== AS5600 health gate (anti-runaway) =====
static uint8_t g_as5600_good[4]     = {0,0,0,0};
static uint8_t g_as5600_fail[4]     = {0,0,0,0};
static uint8_t g_as5600_okstreak[4] = {0,0,0,0};
static constexpr uint8_t kAS5600_FAIL_TRIP   = 3;
static constexpr uint8_t kAS5600_OK_RECOVER  = 2;
static inline bool AS5600_is_good(uint8_t ch) { return g_as5600_good[ch] != 0; }

// ---- liniowe zwalnianie końcówki + minimalny PWM ----
static constexpr float PULL_V_FAST   = 60.0f;   // mm/s
static constexpr float PULL_V_END    = 12.0f;   // mm/s na samym końcu
static constexpr float PULL_RAMP_M   = 0.015f;  // 15mm strefa hamowania
static constexpr float PULL_PWM_MIN  = 400.0f;  // "kop" przy pullback

static float g_pull_remain_m[4]  = {0,0,0,0};
static float g_pull_speed_set[4] = {-PULL_V_FAST,-PULL_V_FAST,-PULL_V_FAST,-PULL_V_FAST}; // mm/s (ujemne)

float MC_PULL_V_OFFSET[4]      = {0.0f, 0.0f, 0.0f, 0.0f};
float MC_PULL_V_MIN[4]         = {1.00f, 1.00f, 1.00f, 1.00f};
float MC_PULL_V_MAX[4]         = {2.00f, 2.00f, 2.00f, 2.00f};
int8_t MC_PULL_POLARITY[4]     = {1, 1, 1, 1};
float MC_DM_KEY_NONE_THRESH[4] = {0.60f, 0.60f, 0.60f, 0.60f};

uint8_t MC_PULL_pct[4]        = {50, 50, 50, 50};
static float MC_PULL_pct_f[4] = {50.0f, 50.0f, 50.0f, 50.0f};

static float  MC_PULL_stu_raw[4]        = {1.65f, 1.65f, 1.65f, 1.65f};
static int8_t MC_PULL_stu[4]            = {0, 0, 0, 0};

static uint8_t  MC_ONLINE_key_stu[4]    = {0, 0, 0, 0};
static uint8_t  g_on_use_low_latch[4]   = {0, 0, 0, 0};   // 1=stop motor latch
static uint8_t  g_on_use_jam_latch[4]   = {0, 0, 0, 0};   // 1=real jam -> 0xF06F
static uint32_t g_on_use_hi_pwm_us[4]   = {0u, 0u, 0u, 0u};

static inline __attribute__((always_inline)) void MC_STU_RGB_set_latch(uint8_t ch, uint8_t r, uint8_t g, uint8_t b, uint64_t now_ms, uint8_t blink)
{
    if (!g_on_use_low_latch[ch]) { MC_STU_RGB_set(ch, r, g, b); return; }

    if (!blink || (((now_ms / 1000ull) & 1ull) != 0ull))
        MC_STU_RGB_set(ch, 0xFFu, 0x00u, 0x00u);
    else
        MC_STU_RGB_set(ch, r, g, b);
}

#if BMCU_DM_TWO_MICROSWITCH
static inline uint8_t dm_key_to_state(uint8_t ch, float v)
{
    const float none_thr = MC_DM_KEY_NONE_THRESH[ch];

    if (v < none_thr) return 0u;   // none
    if (v > 1.7f)     return 1u;   // both
    if (v > 1.4f)     return 2u;   // external only
    return 3u;
}

// ---- DM autoload (two microswitch) ----
static constexpr uint64_t DM_AUTO_S1_DEBOUNCE_MS       = 100ull;   // 0.1s
static constexpr uint64_t DM_AUTO_S1_TIMEOUT_MS        = 5000ull;  // 5s
static constexpr uint64_t DM_AUTO_S1_FAIL_RETRACT_MS   = 1500ull;  // 1.5s

static constexpr float    DM_AUTO_S2_TARGET_M          = 0.120f;   // 120mm
static constexpr float    DM_AUTO_BUF_ABORT_PCT        = 75.0f;    // abort push
static constexpr float    DM_AUTO_BUF_RECOVER_PCT      = 50.2f;    // retract-to (try 1/2)
static constexpr uint64_t DM_AUTO_FAIL_EXTRA_MS        = 1500ull;  // extra retract after fail
static constexpr float    DM_AUTO_PWM_PUSH             = 900.0f;   // push strength
static constexpr float    DM_AUTO_PWM_PULL             = 900.0f;   // retract strength
static constexpr float    DM_AUTO_IDLE_LIM             = 950.0f;   // clamp only during autoload

enum : uint8_t
{
    DM_AUTO_IDLE = 0,
    DM_AUTO_S1_DEBOUNCE,
    DM_AUTO_S1_PUSH,
    DM_AUTO_S1_FAIL_RETRACT,
    DM_AUTO_S2_PUSH,
    DM_AUTO_S2_RETRACT,
    DM_AUTO_S2_FAIL_RETRACT,
    DM_AUTO_S2_FAIL_EXTRA,
};

static uint8_t  dm_loaded[4]            = {1,1,1,1};   // 1=loaded (after stage2 success)
static uint8_t  dm_fail_latch[4]        = {0,0,0,0};   // latch until ks==0 (<0.6V)
static uint8_t  dm_auto_state[4]        = {0,0,0,0};
static uint8_t  dm_autoload_gate[4]     = {0,0,0,0}; // 0=allow Stage1, 1=block Stage1 until idle+ks==0
static uint8_t  dm_auto_try[4]          = {0,0,0,0};   // abort count (stage2)
static uint64_t dm_auto_t0_ms[4]        = {0ull,0ull,0ull,0ull};
static float    dm_auto_remain_m[4]     = {0,0,0,0};
static float    dm_auto_last_m[4]       = {0,0,0,0};

static uint64_t dm_loaded_drop_t0_ms[4] = {0ull,0ull,0ull,0ull};
#endif

static constexpr float    AUTO_UNLOAD_START_PCT      = 80.0f;
static constexpr float    AUTO_UNLOAD_NEUTRAL_LO_PCT = 45.0f;
static constexpr float    AUTO_UNLOAD_NEUTRAL_HI_PCT = 55.0f;
static constexpr float    AUTO_UNLOAD_ABORT_PCT      = 35.0f;
static constexpr uint64_t AUTO_UNLOAD_ARM_MS         = 1000ull;
static constexpr uint64_t AUTO_UNLOAD_MAX_MS         = 15000ull;
static constexpr uint64_t AUTO_UNLOAD_EMPTY_MS       = 1500ull;
static constexpr float    AUTO_UNLOAD_PWM_PULL       = 850.0f;

static uint8_t  auto_unload_arm[4]          = {0,0,0,0};
static uint8_t  auto_unload_active[4]       = {0,0,0,0};
static uint8_t  auto_unload_blocked[4]      = {0,0,0,0};
static uint64_t auto_unload_arm_t0_ms[4]    = {0ull,0ull,0ull,0ull};
static uint64_t auto_unload_active_t0_ms[4] = {0ull,0ull,0ull,0ull};
static uint64_t auto_unload_empty_t0_ms[4]  = {0ull,0ull,0ull,0ull};

bool filament_channel_inserted[4]       = {false, false, false, false}; // czy kanał fizycznie wpięty

static constexpr float MC_PULL_PIDP_PCT = 25.0f;

static constexpr int MC_PULL_DEADBAND_PCT_LOW  = 30;
static constexpr int MC_PULL_DEADBAND_PCT_HIGH = 70;

// ================ LOAD CONTROL ======================
#if BMCU_SOFT_LOAD
    // Stage1
    static constexpr int   MC_LOAD_S1_FAST_PCT       = 75;
    static constexpr int   MC_LOAD_S1_HARD_STOP_PCT  = 90;  // bezpiecznik
    static constexpr int   MC_LOAD_S1_HARD_HYS       = 2;   // wróć dopiero < (HARD_STOP - HYS)
    // Stage2 (hold_load)
    static constexpr float MC_LOAD_S2_HOLD_TARGET_PCT    = 75.0f;
    static constexpr float MC_LOAD_S2_HOLD_BAND_LO_DELTA = 0.3f;   // push_hi = hold_target - delta
    static constexpr float MC_LOAD_S2_PUSH_START_PCT     = 55.0f;  // start push PWM
    static constexpr float MC_LOAD_S2_PWM_HI             = 480.0f;
    static constexpr float MC_LOAD_S2_PWM_LO             = 1000.0f;
    // ===== ON_USE CONTROL =====
    static constexpr float MC_ON_USE_TARGET_PCT    = 52.0f;
    static constexpr float MC_ON_USE_BAND_LO_DELTA = 0.2f;  // band_lo = target - delta
    static constexpr float MC_ON_USE_BAND_HI_PCT   = 60.0f;
#elif BMCU_P1S  // P1S
    // Stage1
    static constexpr int   MC_LOAD_S1_FAST_PCT       = 88;
    static constexpr int   MC_LOAD_S1_HARD_STOP_PCT  = 97;  // bezpiecznik
    static constexpr int   MC_LOAD_S1_HARD_HYS       = 2;   // wróć dopiero < (HARD_STOP - HYS)
    // Stage2 (hold_load)
    static constexpr float MC_LOAD_S2_HOLD_TARGET_PCT    = 95.0f;
    static constexpr float MC_LOAD_S2_HOLD_BAND_LO_DELTA = 1.0f;   // push_hi = hold_target - delta
    static constexpr float MC_LOAD_S2_PUSH_START_PCT     = 88.0f;  // start push PWM
    static constexpr float MC_LOAD_S2_PWM_HI             = 550.0f;
    static constexpr float MC_LOAD_S2_PWM_LO             = 1000.0f;
    // ===== ON_USE CONTROL =====
    static constexpr float MC_ON_USE_TARGET_PCT    = 54.0f;
    static constexpr float MC_ON_USE_BAND_LO_DELTA = 0.2f;  // band_lo = target - delta
    static constexpr float MC_ON_USE_BAND_HI_PCT   = 65.0f;
#else        // A1
    // Stage1
    static constexpr int   MC_LOAD_S1_FAST_PCT       = 85;
    static constexpr int   MC_LOAD_S1_HARD_STOP_PCT  = 95;  // bezpiecznik
    static constexpr int   MC_LOAD_S1_HARD_HYS       = 2;   // wróć dopiero < (HARD_STOP - HYS)
    // Stage2 (hold_load)
    static constexpr float MC_LOAD_S2_HOLD_TARGET_PCT    = 90.0f;
    static constexpr float MC_LOAD_S2_HOLD_BAND_LO_DELTA = 0.3f;   // push_hi = hold_target - delta
    static constexpr float MC_LOAD_S2_PUSH_START_PCT     = 80.0f;  // start push PWM
    static constexpr float MC_LOAD_S2_PWM_HI             = 480.0f;
    static constexpr float MC_LOAD_S2_PWM_LO             = 1000.0f;
    // ===== ON_USE CONTROL =====
    static constexpr float MC_ON_USE_TARGET_PCT    = 52.0f;
    static constexpr float MC_ON_USE_BAND_LO_DELTA = 0.2f;  // band_lo = target - delta
    static constexpr float MC_ON_USE_BAND_HI_PCT   = 60.0f;
#endif
// ====================================================

static constexpr uint32_t CAL_RESET_HOLD_MS     = 5000;
static constexpr int      CAL_RESET_PCT_THRESH  = 15;
static constexpr float    CAL_RESET_V_DELTA     = 0.10f;
static constexpr float    CAL_RESET_NEAR_MIN    = 0.03f;

static int      g_hold_ch = -1;
static uint32_t g_hold_t0_ticks = 0;

// kiedy kanał OSTATNIO wyszedł z on_use (0 = nigdy, 1 = marker "był kiedykolwiek") (patch do wersji BMCU DM przy automatycznej zmianie filamentu gdy się skończy, żeby ekstruder nie trzymał filamentu)
static uint64_t g_last_on_use_exit_ms[4] = {0,0,0,0};

extern void RGB_update();

static inline bool all_no_filament()
{
    return ((MC_ONLINE_key_stu[0] | MC_ONLINE_key_stu[1] | MC_ONLINE_key_stu[2] | MC_ONLINE_key_stu[3]) == 0);
}

static void blink_all_blue_3s()
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = 3000u * tpm;

    while ((uint32_t)(time_ticks32() - t0) < dt)
    {
        const uint32_t now_t = time_ticks32();
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);

        const bool on = (((elapsed_ms / 150u) & 1u) == 0u);
        for (uint8_t ch = 0; ch < kChCount; ch++)
            MC_PULL_ONLINE_RGB_set(ch, 0, 0, on ? 0x10 : 0);

        RGB_update();
        delay(20);
    }

    for (uint8_t ch = 0; ch < kChCount; ch++)
        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
}

static void calibration_reset_and_reboot()
{
    for (uint8_t i = 0; i < kChCount; i++) Motion_control_set_PWM(i, 0);

    blink_all_blue_3s();

    Flash_NVM_full_clear();

    NVIC_SystemReset();
}

static float pull_v_to_percent_f(uint8_t ch, float v)
{
    constexpr float c = 1.65f;

    float vmin = MC_PULL_V_MIN[ch];
    float vmax = MC_PULL_V_MAX[ch];

    if (vmin > 1.60f) vmin = 1.60f;
    if (vmax < 1.70f) vmax = 1.70f;
    if (vmax <= (vmin + 0.10f)) { vmin = 1.55f; vmax = 1.75f; }

    float pos01;
    if (v <= c)
    {
        float den = c - vmin;
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f * (v - vmin) / den;
    }
    else
    {
        float den = vmax - c;
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f + 0.5f * (v - c) / den;
    }

    return clampf(pos01, 0.0f, 1.0f) * 100.0f;
}

static inline float pull_v_apply_polarity(uint8_t ch, float v)
{
    if (MC_PULL_POLARITY[ch] < 0) return 3.30f - v;
    return v;
}

void MC_PULL_detect_channels_inserted()
{
    if (!ADC_DMA_is_inited())
    {
        for (uint8_t ch = 0; ch < kChCount; ch++) filament_channel_inserted[ch] = false;
        return;
    }

    ADC_DMA_gpio_analog();
    ADC_DMA_filter_reset();
    (void)ADC_DMA_wait_full();

    constexpr uint8_t idx[kChCount] = {6,4,2,0};
    constexpr int N = 16;
    float s[kChCount] = {0,0,0,0};

    for (int i = 0; i < N; i++)
    {
        const float *v = ADC_DMA_get_value();
        for (uint8_t ch = 0; ch < kChCount; ch++) s[ch] += v[idx[ch]];
        delay(2);
    }

    constexpr float VMIN = 0.30f;
    constexpr float VMAX = 3.00f;
    constexpr float invN = 1.0f / (float)N;

    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        const float a = s[ch] * invN;
        filament_channel_inserted[ch] = (a > VMIN) && (a < VMAX);
    }
}

static inline void MC_PULL_ONLINE_init()
{
    MC_PULL_detect_channels_inserted();
}

static inline void MC_PULL_ONLINE_read(uint32_t now_ticks)
{
    const float *data = ADC_DMA_get_value();

    // mapowanie ADC -> kanały
    MC_PULL_stu_raw[3] = pull_v_apply_polarity(3u, data[0] + MC_PULL_V_OFFSET[3]);
    const float key3   = data[1];

    MC_PULL_stu_raw[2] = pull_v_apply_polarity(2u, data[2] + MC_PULL_V_OFFSET[2]);
    const float key2   = data[3];

    MC_PULL_stu_raw[1] = pull_v_apply_polarity(1u, data[4] + MC_PULL_V_OFFSET[1]);
    const float key1   = data[5];

    MC_PULL_stu_raw[0] = pull_v_apply_polarity(0u, data[6] + MC_PULL_V_OFFSET[0]);
    const float key0   = data[7];

#if BMCU_DM_TWO_MICROSWITCH
    const float keyv[4] = { key0, key1, key2, key3 };

    // --- Buffer Gesture Load  ---
    static uint32_t gst_t0_ticks[4]     = {0,0,0,0};
    static uint8_t  gst_step[4]         = {0,0,0,0};      // 0=idle, 1=wait_low, 2=wait_return
    static bool     gst_active[4]       = {false,false,false,false};
    static uint32_t gst_act_t0_ticks[4] = {0,0,0,0};

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    const uint32_t T100  = 100u  * tpm;
    const uint32_t T2000 = 2000u * tpm;
    const uint32_t T5500 = 5500u * tpm;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (!filament_channel_inserted[i])
        {
            gst_step[i] = 0;
            gst_active[i] = false;
            gst_t0_ticks[i] = 0;
            gst_act_t0_ticks[i] = 0;
            MC_ONLINE_key_stu[i] = 0u;
            continue;
        }

        if (dm_fail_latch[i])
        {
            gst_step[i] = 0;
            gst_active[i] = false;
        }

        if (!gst_active[i])
        {
            const float pct_f = pull_v_to_percent_f(i, MC_PULL_stu_raw[i]);

            if (gst_step[i] == 0)
            {
                if (pct_f < 10.0f) { gst_step[i] = 1; gst_t0_ticks[i] = now_ticks; }
            }
            else if (gst_step[i] == 1)
            {
                if (pct_f > 15.0f) { gst_step[i] = 0; }
                else if ((uint32_t)(now_ticks - gst_t0_ticks[i]) >= T100)
                {
                    gst_step[i] = 2;
                }
            }
            else
            {
                if ((uint32_t)(now_ticks - gst_t0_ticks[i]) > T2000)
                {
                    gst_step[i] = 0;
                }
                else if (pct_f >= 45.0f && pct_f <= 55.0f)
                {
                    gst_active[i] = true;
                    gst_act_t0_ticks[i] = now_ticks;
                    gst_step[i] = 0;
                }
            }
        }

        if (gst_active[i])
        {
            if (keyv[i] > 1.7f) gst_active[i] = false;
            else if ((uint32_t)(now_ticks - gst_act_t0_ticks[i]) > T5500) gst_active[i] = false;
        }

        const uint8_t phys = dm_key_to_state(i, keyv[i]);
        uint8_t state = phys;

        if (gst_active[i] && (phys == 0u)) state = 2u;

        MC_ONLINE_key_stu[i] = state;
    }
    // --- End Buffer Gesture Load  ---
#else
    // online key: tylko jeśli kanał fizycznie wpięty
    MC_ONLINE_key_stu[3] = (filament_channel_inserted[3] && (key3 > 1.7f)) ? 1u : 0u;
    MC_ONLINE_key_stu[2] = (filament_channel_inserted[2] && (key2 > 1.7f)) ? 1u : 0u;
    MC_ONLINE_key_stu[1] = (filament_channel_inserted[1] && (key1 > 1.7f)) ? 1u : 0u;
    MC_ONLINE_key_stu[0] = (filament_channel_inserted[0] && (key0 > 1.7f)) ? 1u : 0u;
#endif


    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ins = filament_channel_inserted[i];

        // jeśli kanał nie jest wpięty -> neutral
        if (!ins)
        {
            MC_ONLINE_key_stu[i] = 0;
            MC_PULL_pct_f[i] = 50.0f;
            MC_PULL_pct[i]   = 50;
            MC_PULL_stu[i]   = 0;
            continue;
        }

        const float pct_f = pull_v_to_percent_f(i, MC_PULL_stu_raw[i]);
        MC_PULL_pct_f[i] = pct_f;

        int pct = (int)(pct_f + 0.5f);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        MC_PULL_pct[i] = (uint8_t)pct;

        if      (pct > MC_PULL_DEADBAND_PCT_HIGH) MC_PULL_stu[i] = 1;
        else if (pct < MC_PULL_DEADBAND_PCT_LOW)  MC_PULL_stu[i] = -1;
        else                                      MC_PULL_stu[i] = 0;
    }

    // pressure do hosta (tylko dla aktywnego kanału)
    auto &A = ams[motion_control_ams_num];
    const uint8_t num = A.now_filament_num;

    if ((num != 0xFF) && (num < kChCount) && filament_channel_inserted[num])
    {
        const uint8_t pct = MC_PULL_pct[num];
            const uint32_t hi = (pct > 50u) ? (uint32_t)(pct - 50u) : 0u;
            A.pressure = (int)((hi * 65535u) / 50u);
    }
    else
    {
        A.pressure = 0xFFFF;
    }
}

// ===== zapis kierunku silników + progow DM key =====
struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    uint32_t check;
    uint8_t dm_key_none_cv[4];
} Motion_control_data_save;

static inline void Motion_control_defaults()
{
    for (uint8_t i = 0; i < kChCount; i++)
    {
        Motion_control_data_save.Motion_control_dir[i] = 0;
        Motion_control_data_save.dm_key_none_cv[i] = 60u;
    }

    Motion_control_data_save.check = 0x40614061u;
}

static inline void Motion_control_apply_saved()
{
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (Motion_control_data_save.dm_key_none_cv[i] < 60u)
            Motion_control_data_save.dm_key_none_cv[i] = 60u;

        MC_DM_KEY_NONE_THRESH[i] = dm_key_centi_to_v(Motion_control_data_save.dm_key_none_cv[i]);
    }
}

static inline bool Motion_control_read()
{
    Motion_control_defaults();

    if (!Flash_Motion_read(&Motion_control_data_save, (uint16_t)sizeof(Motion_control_save_struct)))
    {
        Motion_control_apply_saved();
        return false;
    }

    if (Motion_control_data_save.check != 0x40614061u)
    {
        Motion_control_defaults();
        Motion_control_apply_saved();
        return false;
    }

    Motion_control_apply_saved();
    return true;
}

static inline bool Motion_control_save()
{
    Motion_control_data_save.check = 0x40614061u;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        uint8_t cv = dm_key_v_to_centi_ceil(MC_DM_KEY_NONE_THRESH[i]);
        if (cv < 60u) cv = 60u;
        Motion_control_data_save.dm_key_none_cv[i] = cv;
    }

    return Flash_Motion_write(&Motion_control_data_save, (uint16_t)sizeof(Motion_control_save_struct));
}

bool Motion_control_save_dm_key_none_thresholds(void)
{
    float thr[4];
    for (uint8_t i = 0; i < kChCount; i++)
        thr[i] = MC_DM_KEY_NONE_THRESH[i];

    (void)Motion_control_read();

    for (uint8_t i = 0; i < kChCount; i++)
        MC_DM_KEY_NONE_THRESH[i] = thr[i];

    return Motion_control_save();
}
// ===== PID =====
class MOTOR_PID
{
    float P = 0;
    float I = 0;
    float D = 0;
    float I_save = 0;
    float E_last = 0;

    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) * 0.5f;

public:
    MOTOR_PID() = default;

    MOTOR_PID(float P_set, float I_set, float D_set)
    {
        init_PID(P_set, I_set, D_set);
    }

    void init_PID(float P_set, float I_set, float D_set)
    {
        P = P_set;
        I = I_set;
        D = D_set;
        I_save = 0;
        E_last = 0;
    }

    float caculate(float E, float time_E)
    {
        I_save += I * E * time_E;
        if (I_save > pid_range)  I_save = pid_range;
        if (I_save < -pid_range) I_save = -pid_range;

        float out;
        if (time_E != 0.0f)
            out = P * E + I_save + D * (E - E_last) / time_E;
        else
            out = P * E + I_save;

        if (out > pid_MAX) out = pid_MAX;
        if (out < pid_MIN) out = pid_MIN;

        E_last = E;
        return out;
    }

    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};

enum class filament_motion_enum
{
    filament_motion_send,
    filament_motion_redetect,
    filament_motion_pull,
    filament_motion_stop,
    filament_motion_before_on_use,
    filament_motion_stop_on_use,
    filament_motion_pressure_ctrl_on_use,
    filament_motion_pressure_ctrl_idle,
    filament_motion_before_pull_back,
};



// ===== Motor control =====
class _MOTOR_CONTROL
{
public:
    filament_motion_enum motion = filament_motion_enum::filament_motion_stop;
    int CHx = 0;

    uint8_t pwm_zeroed = 1;

    uint64_t motor_stop_time = 0;

    float    post_sendout_retract_thresh_pct = -1.0f;
    uint8_t  retract_hys_active = 0;
    float    on_use_hi_gate_pct = -1.0f;
    uint64_t on_use_hi_gate_t0_ms = 0ull;

    uint64_t send_start_ms = 0;
    float    send_start_m  = 0.0f;
    uint8_t  send_len_abort = 0;

    uint64_t pull_start_ms = 0;

    bool send_stop_latch = false;

    MOTOR_PID PID_speed    = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(MC_PULL_PIDP_PCT, 0, 0);

    float pwm_zero = 500;
    float dir = 0;

    static float x_prev[4];

    bool  send_hard = false;

    _MOTOR_CONTROL(int _CHx) : CHx(_CHx) {}

    void set_pwm_zero(float _pwm_zero) { pwm_zero = _pwm_zero; }

    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        set_motion(_motion, over_time, time_ms_fast());
    }

    void set_motion(filament_motion_enum _motion, uint64_t over_time, uint64_t time_now)
    {
        motor_stop_time = (_motion == filament_motion_enum::filament_motion_stop) ? 0 : (time_now + over_time);

        if (motion == _motion) return;

        const filament_motion_enum prev = motion;
        motion = _motion;

        if ((_motion != filament_motion_enum::filament_motion_pressure_ctrl_on_use) &&
            g_on_use_low_latch[CHx] && !g_on_use_jam_latch[CHx])
        {
            g_on_use_low_latch[CHx] = 0u;
            g_on_use_hi_pwm_us[CHx] = 0u;
        }

        pwm_zeroed = 0;

        if (_motion == filament_motion_enum::filament_motion_send) {
            send_start_ms = time_now;
            send_stop_latch = false;
            send_len_abort = 0;
            send_start_m = ams[motion_control_ams_num].filament[CHx].meters;
        }

        if (_motion == filament_motion_enum::filament_motion_pull) {
            pull_start_ms = time_now;
        }

        if (prev == filament_motion_enum::filament_motion_send &&
            _motion != filament_motion_enum::filament_motion_send)
        {
            send_start_ms = 0;
            send_stop_latch = false;
            send_len_abort = 0;
            send_start_m = 0.0f;
        }

        if (prev == filament_motion_enum::filament_motion_pull &&
            _motion != filament_motion_enum::filament_motion_pull)
        {
            pull_start_ms = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            if (g_last_on_use_exit_ms[CHx] == 0) g_last_on_use_exit_ms[CHx] = 1;
        }

        if (prev == filament_motion_enum::filament_motion_pressure_ctrl_on_use &&
            _motion != filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            g_last_on_use_exit_ms[CHx] = time_now;
        }

        if (_motion == filament_motion_enum::filament_motion_send ||
            _motion == filament_motion_enum::filament_motion_pull)
        {
            g_last_on_use_exit_ms[CHx] = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_send)
        {
            send_hard = false;
        }

        if (prev == filament_motion_enum::filament_motion_send &&
            _motion != filament_motion_enum::filament_motion_send)
        {
            send_hard = false;
        }

        PID_speed.clear();
        PID_pressure.clear();

        const bool keep_pwm =
            (prev == filament_motion_enum::filament_motion_send) &&
            (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use);

        if (_motion == filament_motion_enum::filament_motion_send)
        {
            post_sendout_retract_thresh_pct = -1.0f;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_before_on_use || _motion == filament_motion_enum::filament_motion_stop_on_use)
        {
            float p = MC_PULL_pct_f[CHx];
            if (p < 0.0f) p = 0.0f;
            if (p > 100.0f) p = 100.0f;
            post_sendout_retract_thresh_pct = p;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            retract_hys_active = 0;
            float p = MC_PULL_pct_f[CHx];
            if (p < 0.0f) p = 0.0f;
            if (p > 100.0f) p = 100.0f;

            post_sendout_retract_thresh_pct = p;

            if (prev == filament_motion_enum::filament_motion_before_on_use ||
                prev == filament_motion_enum::filament_motion_stop_on_use)
            {
                on_use_hi_gate_pct   = p;
                on_use_hi_gate_t0_ms = time_now;
            }
            else
            {
                on_use_hi_gate_pct   = -1.0f;
                on_use_hi_gate_t0_ms = 0ull;
            }
        }
        else if (prev == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            post_sendout_retract_thresh_pct = -1.0f;
            retract_hys_active   = 0;
            on_use_hi_gate_pct   = -1.0f;
            on_use_hi_gate_t0_ms = 0ull;
        }

        if (_motion == filament_motion_enum::filament_motion_pull)
        {
            post_sendout_retract_thresh_pct = -1.0f;
            retract_hys_active = 0;
        }

        if (!keep_pwm)
        {
            x_prev[CHx] = 0.0f;
        }
        else
        {
            if (x_prev[CHx] > 600.0f)  x_prev[CHx] = 600.0f;
            if (x_prev[CHx] < -850.0f) x_prev[CHx] = -850.0f;
        }
    }

    filament_motion_enum get_motion() { return motion; }

    static inline void hold_load(
        float pct,
        float dir,
        MOTOR_PID &PID_pressure,
        float &post_sendout_retract_thresh_pct,
        uint8_t &retract_hys_active,
        float &x,
        bool  &on_use_need_move,
        float &on_use_abs_err,
        bool  &on_use_linear
    )
    {
        constexpr float hold_target = MC_LOAD_S2_HOLD_TARGET_PCT;

        float thresh = post_sendout_retract_thresh_pct;
        if (thresh < hold_target) thresh = hold_target;

        if (pct > thresh)
        {
            const float target = thresh;

            const float start_retract = target + 0.25f;
            const float stop_retract  = target + 0.00f;

            retract_hys_active = hyst_u8(retract_hys_active, pct, start_retract, stop_retract);

            if (!retract_hys_active)
            {
                x = 0.0f;
                PID_pressure.clear();
                on_use_need_move = false;
                on_use_abs_err   = 0.0f;
                on_use_linear    = false;
            }
            else
            {
                const float err = pct - target;
                on_use_need_move = true;
                on_use_abs_err   = err;
                on_use_linear    = false;

                const float mag = retract_mag_from_err(err, 850.0f);

                x = dir * mag;
                if (x * dir < 0.0f) x = 0.0f;
            }
        }
        else
        {
            retract_hys_active = 0;

            constexpr float push_hi_pct    = hold_target - MC_LOAD_S2_HOLD_BAND_LO_DELTA;
            constexpr float push_start_pct = MC_LOAD_S2_PUSH_START_PCT;
            constexpr float pwm_hi         = MC_LOAD_S2_PWM_HI;
            constexpr float pwm_lo         = MC_LOAD_S2_PWM_LO;
            constexpr float slope          = (pwm_lo - pwm_hi) / (push_hi_pct - push_start_pct);

            if (pct >= push_hi_pct)
            {
                x = 0.0f;
                PID_pressure.clear();
                on_use_need_move = false;
                on_use_abs_err   = 0.0f;
                on_use_linear    = false;
            }
            else
            {
                float pwm;
                if (pct <= push_start_pct) pwm = pwm_lo;
                else                       pwm = pwm_hi + (push_hi_pct - pct) * slope;

                x = -dir * pwm;
                PID_pressure.clear();

                on_use_need_move = true;
                on_use_abs_err   = hold_target - pct;
                on_use_linear    = true;
            }
        }
    }

    void run(float time_E, uint64_t now_ms)
    {
        if (motion == filament_motion_enum::filament_motion_stop &&
            motor_stop_time == 0 &&
            pwm_zeroed)
            return;

        if (motion != filament_motion_enum::filament_motion_stop &&
            motor_stop_time != 0 &&
            now_ms > motor_stop_time)
        {
            if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
                g_last_on_use_exit_ms[CHx] = now_ms;

            PID_speed.clear();
            PID_pressure.clear();
            pwm_zeroed = 1;
            x_prev[CHx] = 0.0f;
            motion = filament_motion_enum::filament_motion_stop;
            Motion_control_set_PWM(CHx, 0);
            return;
        }

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use && g_on_use_low_latch[CHx])
        {
            g_on_use_hi_pwm_us[CHx] = 0u;
            PID_speed.clear();
            PID_pressure.clear();
            pwm_zeroed = 1;
            x_prev[CHx] = 0.0f;
            Motion_control_set_PWM(CHx, 0);
            return;
        }

        float speed_set = 0.0f;
        const float now_speed = speed_as5600[CHx];
        float x = 0.0f;
#if BMCU_DM_TWO_MICROSWITCH
        bool  dm_autoload_active = false;
        float dm_autoload_x      = 0.0f;
#endif

        // info o ostatnim wyjściu z on_use
        const uint64_t t_exit  = g_last_on_use_exit_ms[CHx];
        const bool had_on_use  = (t_exit != 0);
        const bool has_exit_ts = (t_exit > 1);
        uint64_t dt_exit = 0;
        if (has_exit_ts) dt_exit = (now_ms - t_exit);

        // aktywne tylko: idle + brak filamentu + kanał wpięty + kiedykolwiek był w on_use
        const bool post_on_use_active =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) &&
            (MC_ONLINE_key_stu[CHx] == 0) &&
            filament_channel_inserted[CHx] &&
            had_on_use;

        const bool post_on_use_10s  = post_on_use_active && has_exit_ts && (dt_exit < 10000ull);

        const bool on_use_like =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) ||
            (motion == filament_motion_enum::filament_motion_before_on_use) ||
            (motion == filament_motion_enum::filament_motion_stop_on_use) ||
            post_on_use_10s ||
            ((motion == filament_motion_enum::filament_motion_send) && send_stop_latch);

        bool  on_use_need_move = false;
        float on_use_abs_err   = 0.0f;
        bool  on_use_linear    = false;

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)
        {
        #if BMCU_DM_TWO_MICROSWITCH
                    // --- DM autoload (Stage1 + Stage2) ---
                    if (filament_channel_inserted[CHx] && (dm_loaded[CHx] == 0u))
                    {
                        const uint8_t ks = MC_ONLINE_key_stu[CHx];
                        auto &A = ams[motion_control_ams_num];
                        const float cur_m = A.filament[CHx].meters;

                        if (dm_fail_latch[CHx])
                        {
                            dm_autoload_active = true;
                            dm_autoload_x = 0.0f;
                            MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                        }
                        else
                        {
                            if (dm_auto_state[CHx] == DM_AUTO_IDLE)
                            {
                                if (ks == 2u)
                                {
                                    if (dm_autoload_gate[CHx] == 0u)
                                    {
                                        dm_autoload_gate[CHx] = 1u;
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                }
                                else if (ks == 1u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_S2_PUSH;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = DM_AUTO_S2_TARGET_M;
                                    dm_auto_last_m[CHx]   = cur_m;
                                }
                            }

                            switch (dm_auto_state[CHx])
                            {
                            case DM_AUTO_S1_DEBOUNCE:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks != 2u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_DEBOUNCE_MS)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_S1_PUSH;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                break;

                            case DM_AUTO_S1_PUSH:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if (ks == 1u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_S2_PUSH;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = DM_AUTO_S2_TARGET_M;
                                    dm_auto_last_m[CHx]   = cur_m;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_TIMEOUT_MS)
                                {
                                    dm_fail_latch[CHx] = 1u;
                                    dm_auto_state[CHx] = DM_AUTO_S1_FAIL_RETRACT;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                else
                                {
                                    dm_autoload_x = -dir * DM_AUTO_PWM_PUSH;
                                }
                                break;

                            case DM_AUTO_S1_FAIL_RETRACT:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_FAIL_RETRACT_MS)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_PUSH:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks != 1u)
                                {
                                    if (ks == 2u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                        dm_auto_try[CHx]      = 0u;
                                        dm_auto_remain_m[CHx] = 0.0f;
                                        dm_auto_t0_ms[CHx]    = 0ull;
                                    }
                                    break;
                                }

                                // remain -= moved
                                {
                                    const float moved = absf(cur_m - dm_auto_last_m[CHx]);
                                    dm_auto_last_m[CHx] = cur_m;

                                    float r = dm_auto_remain_m[CHx] - moved;
                                    if (r < 0.0f) r = 0.0f;
                                    dm_auto_remain_m[CHx] = r;
                                }

                                if (MC_PULL_pct_f[CHx] > DM_AUTO_BUF_ABORT_PCT)
                                {
                                    uint8_t t = dm_auto_try[CHx];
                                    if (t < 255u) t++;
                                    dm_auto_try[CHx] = t;

                                    dm_auto_last_m[CHx] = cur_m;

                                    if (t >= 3u)
                                    {
                                        dm_fail_latch[CHx] = 1u;
                                        dm_auto_state[CHx] = DM_AUTO_S2_FAIL_RETRACT;
                                    }
                                    else
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S2_RETRACT;
                                    }

                                    MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                                }
                                else if (dm_auto_remain_m[CHx] <= 0.0f)
                                {
                                    dm_loaded[CHx] = 1u;

                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;

                                    MC_STU_RGB_set(CHx, STU_OK_R, STU_OK_G, STU_OK_B);
                                    dm_autoload_x = 0.0f;
                                }
                                else
                                {
                                    dm_autoload_x = -dir * DM_AUTO_PWM_PUSH;
                                }
                                break;

                            case DM_AUTO_S2_RETRACT:
                                dm_autoload_active = true;

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                    break;
                                }

                                // remain += moved
                                {
                                    const float moved = absf(cur_m - dm_auto_last_m[CHx]);
                                    dm_auto_last_m[CHx] = cur_m;

                                    float r = dm_auto_remain_m[CHx] + moved;
                                    if (r > DM_AUTO_S2_TARGET_M) r = DM_AUTO_S2_TARGET_M;
                                    dm_auto_remain_m[CHx] = r;
                                }

                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if ((MC_PULL_pct_f[CHx] <= DM_AUTO_BUF_RECOVER_PCT) || (ks == 2u))
                                {
                                    dm_auto_last_m[CHx] = cur_m;

                                    if (ks == 1u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S2_PUSH;
                                    }
                                    else if (ks == 2u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                        dm_auto_try[CHx]      = 0u;
                                        dm_auto_remain_m[CHx] = 0.0f;
                                        dm_auto_t0_ms[CHx]    = 0ull;
                                    }
                                    dm_autoload_x = 0.0f;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_FAIL_RETRACT:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                }
                                else if (ks == 2u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_S2_FAIL_EXTRA;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_FAIL_EXTRA:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_FAIL_EXTRA_MS)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            default:
                                dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                dm_auto_try[CHx]      = 0u;
                                dm_auto_remain_m[CHx] = 0.0f;
                                dm_auto_t0_ms[CHx]    = 0ull;
                                break;
                            }
                        }
                    }

                    if (dm_autoload_active)
                    {
                        x = dm_autoload_x;
                        PID_pressure.clear();
                        PID_speed.clear();
                    }
                    else
        #endif

            if (MC_ONLINE_key_stu[CHx] == 0)
            {
                if (!filament_channel_inserted[CHx] || !had_on_use)
                {
                    PID_pressure.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0.0f;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }

                if (post_on_use_10s)
                {
                    if ((uint8_t)MC_PULL_pct[CHx] >= 49u)
                    {
                        x = 0.0f;
                        PID_pressure.clear();
                        on_use_need_move = false;
                        on_use_abs_err = 0.0f;
                    }
                    else
                    {
                        const float pct = MC_PULL_pct_f[CHx];
                        const float err = pct - 49.0f;

                        on_use_need_move = true;
                        on_use_abs_err   = -err;

                        x = dir * PID_pressure.caculate(err, time_E);

                        float lim_f = 500.0f + 80.0f * on_use_abs_err;
                        if (lim_f > 900.0f) lim_f = 900.0f;

                        if (x >  lim_f) x =  lim_f;
                        if (x < -lim_f) x = -lim_f;
                        if (x * dir > 0.0f)
                        {
                            x = 0.0f;
                            PID_pressure.clear();
                            on_use_need_move = false;
                            on_use_abs_err   = 0.0f;
                        }
                    }
                }
                else
                {
                    // po 10s: idle jakby filament był -> tylko na krańcach (MC_PULL_stu != 0)
                    if (MC_PULL_stu[CHx] != 0)
                    {
                        const float pct = MC_PULL_pct_f[CHx];
                        x = dir * PID_pressure.caculate(pct - 50.0f, time_E);
                    }
                    else
                    {
                        x = 0.0f;
                        PID_pressure.clear();
                    }
                }
            }
            else
            {
                // normalny idle z filamentem
                if (MC_PULL_stu[CHx] != 0)
                {
                    const float pct = MC_PULL_pct_f[CHx];
                    x = dir * PID_pressure.caculate(pct - 50.0f, time_E);
                }
                else
                {
                    x = 0.0f;
                    PID_pressure.clear();
                }
            }
        }
        else if (motion == filament_motion_enum::filament_motion_redetect) // wyjście do braku filamentu -> ponowne podanie
        {
            x = -dir * 900.0f;
        }
        else if (MC_ONLINE_key_stu[CHx] != 0) // kanał aktywny i jest filament
        {
            if (motion == filament_motion_enum::filament_motion_before_pull_back)
            {
                const float pct = MC_PULL_pct_f[CHx];
                constexpr float target = 50.0f;

                const float start_retract = target + 0.25f;
                const float stop_retract  = target + 0.00f;

                static uint8_t pb_active[4] = {0,0,0,0};

                pb_active[CHx] = hyst_u8(pb_active[CHx], pct, start_retract, stop_retract);

                if (!pb_active[CHx])
                {
                    x = 0.0f;
                    on_use_need_move = false;
                    on_use_abs_err   = 0.0f;
                }
                else
                {
                    const float err = pct - target; // dodatni
                    on_use_need_move = true;
                    on_use_abs_err   = err;

                    const float mag = retract_mag_from_err(err, 850.0f);

                    x = dir * mag;          // tylko cofanie
                    if (x * dir < 0.0f) x = 0.0f;
                }
            }
            else if (motion == filament_motion_enum::filament_motion_before_on_use)
            {
                const float pct = MC_PULL_pct_f[CHx];

                hold_load(
                    pct,
                    dir,
                    PID_pressure,
                    post_sendout_retract_thresh_pct,
                    retract_hys_active,
                    x,
                    on_use_need_move,
                    on_use_abs_err,
                    on_use_linear
                );
            }
            else if (motion == filament_motion_enum::filament_motion_stop_on_use)
            {
                PID_pressure.clear();
                pwm_zeroed = 1;
                x_prev[CHx] = 0.0f;
                Motion_control_set_PWM(CHx, 0);
                return;
            }
            else if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
            {
                const float pct = MC_PULL_pct_f[CHx];

                constexpr float target_pct = MC_ON_USE_TARGET_PCT;
                constexpr float band_hi    = MC_ON_USE_BAND_HI_PCT;

                float band_hi_eff = band_hi;

                if (on_use_hi_gate_pct >= 0.0f)
                {
                    const bool gate_active =
                        (on_use_hi_gate_t0_ms != 0ull) &&
                        ((now_ms - on_use_hi_gate_t0_ms) < 5000ull);

                    if (!gate_active)
                    {
                        on_use_hi_gate_pct   = -1.0f;
                        on_use_hi_gate_t0_ms = 0ull;
                    }
                    else
                    {
                        if (pct > on_use_hi_gate_pct) on_use_hi_gate_pct = pct;

                        const float d = on_use_hi_gate_pct - pct;
                        if (d > 2.0f)
                        {
                            float ng = pct + 1.0f;
                            if (ng < band_hi) ng = band_hi;
                            if (ng > 100.0f)  ng = 100.0f;
                            on_use_hi_gate_pct = ng;
                        }

                        if (on_use_hi_gate_pct > band_hi_eff) band_hi_eff = on_use_hi_gate_pct;
                    }
                }

                constexpr float pwm_lo          = 380.0f;
                constexpr float pct_fast_onuse  = 50.0f;
                constexpr float pwm_fast_onuse  = 900.0f;
                constexpr float pwm_cap         = 900.0f;

                constexpr float slope =
                    (pwm_fast_onuse - pwm_lo) / ((target_pct - MC_ON_USE_BAND_LO_DELTA) - pct_fast_onuse);

                retract_hys_active = 0;

                if (pct >= (target_pct - MC_ON_USE_BAND_LO_DELTA) && pct <= band_hi_eff)
                {
                    x = 0.0f;
                    PID_pressure.clear();
                    on_use_need_move = false;
                    on_use_abs_err   = 0.0f;
                }
                else if (pct < (target_pct - MC_ON_USE_BAND_LO_DELTA))
                {
                    const float err = pct - target_pct;
                    on_use_need_move = true;
                    on_use_abs_err   = -err;

                    float pwm;
                    if (pct >= pct_fast_onuse)
                        pwm = pwm_lo + ((target_pct - MC_ON_USE_BAND_LO_DELTA) - pct) * slope;
                    else
                        pwm = pwm_fast_onuse + (pct_fast_onuse - pct) * slope;

                    if (pwm > pwm_cap) pwm = pwm_cap;

                    x = -dir * pwm;
                    PID_pressure.clear();
                    on_use_linear = true;
                }
                else
                {
                    on_use_need_move = true;

                    const float err = pct - target_pct;
                    on_use_abs_err = (err < 0.0f) ? -err : err;

                    x = dir * PID_pressure.caculate(err, time_E);

                    float lim_f = 500.0f + 80.0f * on_use_abs_err;
                    if (lim_f > 900.0f) lim_f = 900.0f;

                    if (x >  lim_f) x =  lim_f;
                    if (x < -lim_f) x = -lim_f;

                    constexpr float retrig = 55.0f;
                    if (err > 0.0f && pct >= retrig)
                    {
                        float mul = 1.0f + 0.5f * (pct - retrig);
                        if (mul > 3.0f) mul = 3.0f;
                        x *= mul;
                        if (x >  950.0f) x =  950.0f;
                        if (x < -950.0f) x = -950.0f;
                    }
                }
            }
            else
            {
                if (motion == filament_motion_enum::filament_motion_stop)
                {
                    PID_speed.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0.0f;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }

                bool do_speed_pid = true;

                if (motion == filament_motion_enum::filament_motion_send)
                {
                    const float pct = MC_PULL_pct_f[CHx];

                    if (!send_len_abort)
                    {
                        constexpr float SEND_MAX_M = 10.0f;
                        const float moved_m = absf(ams[motion_control_ams_num].filament[CHx].meters - send_start_m);
                        if (moved_m >= SEND_MAX_M) send_len_abort = 1;
                    }

                    if (send_len_abort)
                    {
                        PID_speed.clear();
                        PID_pressure.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0.0f;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    // HARD STOP
                    if (pct >= (float)MC_LOAD_S1_HARD_STOP_PCT)
                    {
                        send_hard = true;
                        PID_speed.clear();
                        PID_pressure.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0.0f;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    if (send_hard)
                    {
                        if (pct >= (float)(MC_LOAD_S1_HARD_STOP_PCT - MC_LOAD_S1_HARD_HYS))
                        {
                            PID_speed.clear();
                            PID_pressure.clear();
                            pwm_zeroed = 1;
                            x_prev[CHx] = 0.0f;
                            Motion_control_set_PWM(CHx, 0);
                            return;
                        }
                        send_hard = false;
                    }

                    if (!send_stop_latch && (pct >= (float)MC_LOAD_S1_FAST_PCT))
                    {
                        send_stop_latch = true;

                        float p = pct;
                        if (p < 0.0f) p = 0.0f;
                        if (p > 100.0f) p = 100.0f;

                        post_sendout_retract_thresh_pct = p;
                        retract_hys_active = 0;

                        PID_speed.clear();
                        PID_pressure.clear();
                    }

                    if (send_stop_latch)
                    {
                        do_speed_pid = false;

                        hold_load(
                            pct,
                            dir,
                            PID_pressure,
                            post_sendout_retract_thresh_pct,
                            retract_hys_active,
                            x,
                            on_use_need_move,
                            on_use_abs_err,
                            on_use_linear
                        );
                    }
                    else
                    {
                        constexpr uint64_t SEND_SOFTSTART_MS = 300ull;
                        constexpr float    V0 = 10.0f;
                        constexpr float    V  = 60.0f;

                        const uint64_t dt = (send_start_ms != 0) ? (now_ms - send_start_ms) : 1000000ull;

                        if (dt < SEND_SOFTSTART_MS)
                        {
                            float t = (float)dt / (float)SEND_SOFTSTART_MS;
                            if (t < 0.0f) t = 0.0f;
                            if (t > 1.0f) t = 1.0f;
                            speed_set = V0 + (V - V0) * t;
                        }
                        else
                        {
                            speed_set = V;
                        }
                    }
                }

                if (motion == filament_motion_enum::filament_motion_pull) // cofanie
                {
                    speed_set = g_pull_speed_set[CHx]; // dynamiczne (liniowo w końcówce)
                }

                if (do_speed_pid)
                    x = dir * PID_speed.caculate(now_speed - speed_set, time_E);
            }
        }
        else
        {
            x = 0.0f;
        }

        // stałe tryby
        const bool pull_mode = (motion == filament_motion_enum::filament_motion_pull);
        const bool pb_mode = (motion == filament_motion_enum::filament_motion_before_pull_back);

        const bool send_stop_hold_mode =
            (motion == filament_motion_enum::filament_motion_send) && send_stop_latch;

        const bool hold_mode =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) ||
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) ||
            (motion == filament_motion_enum::filament_motion_before_on_use) ||
            (motion == filament_motion_enum::filament_motion_stop_on_use) ||
            post_on_use_active ||
            send_stop_hold_mode;

        const int deadband =
            pb_mode ? 0 :
            (hold_mode ? 1 : (pull_mode ? 2 : 10));

        float pwm0 =
            pb_mode ? 0.0f :
            (hold_mode ? 420.0f : pwm_zero);

        if (pull_mode)
        {
            float k = g_pull_remain_m[CHx] / PULL_RAMP_M;
            k = clampf(k, 0.0f, 1.0f);

            // daleko: ~pwm_zero (500), przy końcu: >=400
            pwm0 = PULL_PWM_MIN + (pwm_zero - PULL_PWM_MIN) * k;

            if (pwm0 < PULL_PWM_MIN) pwm0 = PULL_PWM_MIN;
        }

        if (x > (float)deadband)
        {
            if (x < pwm0) x = pwm0;
        }
        else if (x < (float)-deadband)
        {
            if (-x < pwm0) x = -pwm0;
        }
        else
        {
            x = 0.0f;
        }

        // clamp
        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)
        {
        #if BMCU_DM_TWO_MICROSWITCH
            const float lim = dm_autoload_active ? DM_AUTO_IDLE_LIM : 800.0f;
            if (x >  lim) x =  lim;
            if (x < -lim) x = -lim;
        #else
            constexpr float PWM_IDLE_LIM = 800.0f;
            if (x >  PWM_IDLE_LIM) x =  PWM_IDLE_LIM;
            if (x < -PWM_IDLE_LIM) x = -PWM_IDLE_LIM;
        #endif
        }
        else
        {
            if (x >  (float)PWM_lim) x =  (float)PWM_lim;
            if (x < (float)-PWM_lim) x = (float)-PWM_lim;
        }

        // ON_USE: min PWM + anty-stall
        static float    stall_s[4] = {0,0,0,0};
        static uint64_t block_until_ms[4] = {0,0,0,0};

        if (on_use_like)
        {
            if (now_ms < block_until_ms[CHx])
            {
                PID_pressure.clear();
                pwm_zeroed = 1;
                x_prev[CHx] = 0.0f;
                Motion_control_set_PWM(CHx, 0);
                return;
            }

            if (on_use_need_move && x != 0.0f)
            {
                if (!on_use_linear)
                {
                    const int MIN_MOVE_PWM = (on_use_abs_err >= 1.3f) ? 500 : 0;
                    if (MIN_MOVE_PWM)
                    {
                        int xi = (int)(x + ((x >= 0.0f) ? 0.5f : -0.5f));
                        const int ax = (xi < 0) ? -xi : xi;
                        if (ax < MIN_MOVE_PWM)
                            x = (x > 0.0f) ? (float)MIN_MOVE_PWM : (float)-MIN_MOVE_PWM;
                    }
                }
            }

            const bool motor_not_moving = (absf(now_speed) < 1.0f);

            if (on_use_need_move && motor_not_moving && (on_use_abs_err >= 2.0f) && (absf(x) >= 450.0f))
            {
                stall_s[CHx] += time_E;

                if (stall_s[CHx] > 0.15f)
                {
                    const float KICK_PWM = 850.0f;
                    x = (x > 0.0f) ? KICK_PWM : -KICK_PWM;
                }

                if (stall_s[CHx] > 0.8f)
                {
                    stall_s[CHx] = 0.0f;
                    block_until_ms[CHx] = now_ms + 500;
                    PID_pressure.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0.0f;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
            }
            else
            {
                stall_s[CHx] = 0.0f;
            }
        }
        else
        {
            stall_s[CHx] = 0.0f;
            block_until_ms[CHx] = 0ull;
        }

        if (motion == filament_motion_enum::filament_motion_redetect)
        {
            const int pwm_out = (int)x;
            pwm_zeroed = (pwm_out == 0);
            x_prev[CHx] = x;
            Motion_control_set_PWM(CHx, pwm_out);
            return;
        }

        const bool use_ramping =
            ((motion == filament_motion_enum::filament_motion_send) && !send_stop_latch) ||
            (motion == filament_motion_enum::filament_motion_pull);

        if (use_ramping)
        {
            const bool pull_soft_start =
                (motion == filament_motion_enum::filament_motion_pull) &&
                (pull_start_ms != 0) &&
                ((now_ms - pull_start_ms) < 400ull);

            float rate_up   = 4500.0f;
            float rate_down = 6500.0f;

            if (pull_soft_start) rate_up = 2500.0f;

            if (motion == filament_motion_enum::filament_motion_send)
            {
                rate_down = 25000.0f;
                rate_up   = 18000.0f;
            }

            const float max_step_up   = rate_up   * time_E;
            const float max_step_down = rate_down * time_E;

            const float prev = x_prev[CHx];
            const float lo = prev - max_step_down;
            const float hi = prev + max_step_up;

            if (x < lo) x = lo;
            if (x > hi) x = hi;
        }

        const int pwm_out0 = (int)x;

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use && !g_on_use_low_latch[CHx])
        {
            if (MC_ONLINE_key_stu[CHx] == 0u)
            {
                g_on_use_hi_pwm_us[CHx] = 0u;
            }
            else
            {
                const float pct = MC_PULL_pct_f[CHx];

                if (pct < 40.0f)
                {
                    g_on_use_low_latch[CHx] = 1u;
                    g_on_use_jam_latch[CHx] = 1u;
                }
                else
                {
                    const int pwm_cmd = pwm_out0;
                    const int ax = (pwm_cmd < 0) ? -pwm_cmd : pwm_cmd;

                    const bool push_hi =
                        (dir != 0.0f) &&
                        (((float)pwm_cmd) * dir < 0.0f) &&
                        (ax > 800);

                    if (push_hi)
                    {
                        const uint32_t add_us = (uint32_t)(time_E * 1000000.0f + 0.5f);

                        uint32_t t1 = g_on_use_hi_pwm_us[CHx] + add_us;
                        if (t1 > 20000000u) t1 = 20000000u;
                        g_on_use_hi_pwm_us[CHx] = t1;

                        if (t1 >= 20000000u)
                        {
                            g_on_use_low_latch[CHx] = 1u;
                            g_on_use_jam_latch[CHx] = 0u;
                        }
                    }
                    else
                    {
                        g_on_use_hi_pwm_us[CHx] = 0u;
                    }
                }

                if (g_on_use_low_latch[CHx])
                {
                    g_on_use_hi_pwm_us[CHx] = 0u;

                    auto &A = ams[motion_control_ams_num];
                    if (g_on_use_jam_latch[CHx] && A.now_filament_num == (uint8_t)CHx)
                        A.pressure = 0xF06Fu;

                    MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                    PID_speed.clear();
                    PID_pressure.clear();
                    pwm_zeroed = 1;
                    x_prev[CHx] = 0.0f;
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
            }
        }
        else
        {
            g_on_use_hi_pwm_us[CHx] = 0u;
        }

        const int pwm_out = pwm_out0;
        pwm_zeroed = (pwm_out == 0);
        x_prev[CHx] = x;
        Motion_control_set_PWM(CHx, pwm_out);
    }
};

_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};
float _MOTOR_CONTROL::x_prev[4] = {0,0,0,0};

void Motion_control_set_PWM(uint8_t CHx, int PWM)
{
    uint16_t set1 = 0, set2 = 0;

    if (PWM > 0)       set1 = (uint16_t)PWM;
    else if (PWM < 0)  set2 = (uint16_t)(-PWM);
    else { set1 = 1000; set2 = 1000; }

    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    default:
        break;
    }
}

// ===== AS5600 distance/speed =====
int32_t as5600_distance_save[4] = {0,0,0,0};

void AS5600_distance_updata(uint32_t now_ticks)
{
    static uint32_t last_ticks = 0u;
    static uint32_t last_poll_ticks = 0u;
    static uint8_t  have_last_ticks = 0u;
    static uint8_t  was_ok[4] = {0,0,0,0};
    static uint32_t last_stu_ticks = 0u;

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    uint32_t tpus = time_hw_tpus;
    if (!tpus) tpus = 1u;

    uint32_t min_poll_ticks = tpm;
    if ((uint32_t)(now_ticks - last_poll_ticks) < min_poll_ticks)
        return;

    last_poll_ticks = now_ticks;

    if ((uint32_t)(now_ticks - last_stu_ticks) >= (200u * tpm))
    {
        last_stu_ticks = now_ticks;
        MC_AS5600.updata_stu();
    }

    if (!have_last_ticks)
    {
        last_ticks = now_ticks;
        have_last_ticks = 1u;
        return;
    }

    const uint32_t dt_ticks = (uint32_t)(now_ticks - last_ticks);
    if (dt_ticks == 0u) return;
    last_ticks = now_ticks;

    const float inv_dt = (1000000.0f * (float)tpus) / (float)dt_ticks;

    MC_AS5600.updata_angle();
    auto &A = ams[motion_control_ams_num];

    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ok_now = MC_AS5600.online[i] && (MC_AS5600.magnet_stu[i] != AS5600_soft_IIC_many::offline);

        if (ok_now)
        {
            g_as5600_fail[i] = 0;
            if (g_as5600_okstreak[i] < 255u) g_as5600_okstreak[i]++;
            if (g_as5600_okstreak[i] >= kAS5600_OK_RECOVER) g_as5600_good[i] = 1u;
        }
        else
        {
            g_as5600_okstreak[i] = 0u;
            if (g_as5600_fail[i] < 255u) g_as5600_fail[i]++;
            if (g_as5600_fail[i] >= kAS5600_FAIL_TRIP) g_as5600_good[i] = 0u;
        }

        if (!AS5600_is_good(i))
        {
            was_ok[i] = 0u;
            speed_as5600[i] = 0.0f;
            continue;
        }

        if (!was_ok[i])
        {
            as5600_distance_save[i] = MC_AS5600.raw_angle[i];
            speed_as5600[i] = 0.0f;
            was_ok[i] = 1u;
            continue;
        }

        const int32_t last = as5600_distance_save[i];
        const int32_t now  = MC_AS5600.raw_angle[i];

        int32_t diff = now - last;
        if (diff > 2048) diff -= 4096;
        if (diff < -2048) diff += 4096;

        as5600_distance_save[i] = now;

        const float dist_mm = (float)diff * kAS5600_MM_PER_CNT;
        speed_as5600[i] = dist_mm * inv_dt;
        A.filament[i].meters += dist_mm * 0.001f;
    }
}

// ===== stany logiki filamentu =====
enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_before_pull_back,
    filament_pulling_back,
    filament_redetect,
};

static filament_now_position_enum filament_now_position[4];
static float filament_pull_back_meters[4];

static float filament_pull_back_target[4] = {
    motion_control_pull_back_distance,
    motion_control_pull_back_distance,
    motion_control_pull_back_distance,
    motion_control_pull_back_distance
};

// BEFORE_PULLBACK: zapis realnie "wycofanej" drogi (m) (sumowanie całego wycofania)
static float  before_pb_last_m[4]      = {0,0,0,0};
static float  before_pb_retracted_m[4] = {0,0,0,0};
static int8_t before_pb_sign[4]        = {0,0,0,0};

static bool motor_motion_filamnet_pull_back_to_online_key(uint64_t time_now)
{
    bool wait = false;
    auto &A = ams[motion_control_ams_num];

    for (uint8_t i = 0; i < kChCount; i++)
    {
        switch (filament_now_position[i])
        {
        case filament_pulling_back:
        {
            MC_STU_RGB_set_latch(i, 0xFFu, 0x00u, 0xFFu, time_now, 1u);

            const float target = filament_pull_back_target[i];
            const float d = absf(A.filament[i].meters - filament_pull_back_meters[i]);

            if (target <= 0.0f || d >= target)
            {
                g_pull_remain_m[i]  = 0.0f;
                g_pull_speed_set[i] = -PULL_V_FAST;
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                filament_pull_back_target[i] = motion_control_pull_back_distance;
                filament_now_position[i] = filament_redetect;
            }
            else if (MC_ONLINE_key_stu[i] == 0)
            {
                g_pull_remain_m[i]  = 0.0f;
                g_pull_speed_set[i] = -PULL_V_FAST;
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                filament_pull_back_target[i] = motion_control_pull_back_distance;
                filament_now_position[i] = filament_redetect;
            }
            else
            {
                const float remain = target - d; // m (>=0)
                g_pull_remain_m[i] = (remain > 0.0f) ? remain : 0.0f;

                float k = g_pull_remain_m[i] / PULL_RAMP_M;   // 1..0 w końcówce
                k = clampf(k, 0.0f, 1.0f);

                const float v = PULL_V_END + (PULL_V_FAST - PULL_V_END) * k; // mm/s
                g_pull_speed_set[i] = -v;

                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pull, 100, time_now);
            }

            wait = true;
            break;
        }

        case filament_redetect:
        {
            MC_STU_RGB_set_latch(i, 0xFFu, 0xFFu, 0x00u, time_now, 0u);

            if (MC_ONLINE_key_stu[i] == 0)
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_redetect, 100, time_now);
            }
            else
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                filament_now_position[i] = filament_idle;

                A.filament_use_flag = 0x00;
                A.filament[i].motion = _filament_motion::idle;
            }

            wait = true;
            break;
        }

        default:
            break;
        }
    }

    return wait;
}

static void motor_motion_switch(uint64_t time_now)
{
    auto &A = ams[motion_control_ams_num];

    const uint8_t num = A.now_filament_num;
    const _filament_motion motion = (num < kChCount) ? A.filament[num].motion : _filament_motion::idle;

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (i != num)
        {
            filament_now_position[i] = filament_idle;

            if (filament_channel_inserted[i] && (MC_ONLINE_key_stu[i] != 0 || g_last_on_use_exit_ms[i] != 0))
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 1000, time_now);
            else
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 1000, time_now);

            continue;
        }

        if (num >= kChCount) continue;

        if (MC_ONLINE_key_stu[num] != 0)
        {
            switch (motion)
            {
            case _filament_motion::before_on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_before_on_use, 300, time_now);
                MC_STU_RGB_set_latch(num, 0xFFu, 0xFFu, 0x00u, time_now, 0u);
                break;
            }

            case _filament_motion::stop_on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop_on_use, 300, time_now);
                MC_STU_RGB_set_latch(num, 0xFFu, 0x00u, 0x00u, time_now, 0u);
                break;
            }

            case _filament_motion::send_out:
            {
                if (g_on_use_jam_latch[num])
                {
                    if (MC_PULL_pct_f[num] > 85.0f)
                    {
                        g_on_use_low_latch[num] = 0u;
                        g_on_use_jam_latch[num] = 0u;
                        g_on_use_hi_pwm_us[num] = 0u;
                    }
                    else
                    {
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                        MC_STU_RGB_set_latch(num, 0x00u, 0xD5u, 0x2Au, time_now, 0u);
                        break;
                    }
                }

                MC_STU_RGB_set_latch(num, 0x00u, 0xD5u, 0x2Au, time_now, 0u);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100, time_now);
                break;
            }

            case _filament_motion::pull_back:
            {
                MC_STU_RGB_set_latch(num, 0xA0u, 0x2Du, 0xFFu, time_now, 1u);
                filament_now_position[num] = filament_pulling_back;

                filament_pull_back_meters[num] = A.filament[num].meters;

                float target;
                if (g_on_use_jam_latch[num])
                {
                    target = 0.100f;
                }
                else
                {
                    const float already = before_pb_retracted_m[num];
                    target = motion_control_pull_back_distance - already;

                    if (target < 0.0f) target = 0.0f;
                    if (target > motion_control_pull_back_distance) target = motion_control_pull_back_distance;
                }

                filament_pull_back_target[num] = target;

                g_pull_remain_m[num]  = target;
                g_pull_speed_set[num] = -PULL_V_FAST;

                before_pb_retracted_m[num] = 0.0f;
                before_pb_sign[num]        = 0;
                before_pb_last_m[num]      = filament_pull_back_meters[num];

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100, time_now);
                break;
            }

            case _filament_motion::before_pull_back:
            {
                MC_STU_RGB_set_latch(num, 0xFFu, 0xA0u, 0x00u, time_now, 1u);

                if (filament_now_position[num] != filament_before_pull_back)
                {
                    filament_now_position[num] = filament_before_pull_back;
                    before_pb_last_m[num]      = A.filament[num].meters;
                    before_pb_retracted_m[num] = 0.0f;
                    before_pb_sign[num]        = 0;
                }

                {
                    const float m  = A.filament[num].meters;
                    const float dm = m - before_pb_last_m[num];
                    before_pb_last_m[num] = m;

                    const float pct = MC_PULL_pct_f[num];
                    const bool want_retract = (pct > 50.25f);

                    if (want_retract)
                    {
                        if (before_pb_sign[num] == 0 && absf(dm) > 0.0005f)
                            before_pb_sign[num] = (dm >= 0.0f) ? 1 : -1;

                        if (before_pb_sign[num] > 0) {
                            if (dm > 0.0f) before_pb_retracted_m[num] += dm;
                        } else if (before_pb_sign[num] < 0) {
                            if (dm < 0.0f) before_pb_retracted_m[num] += -dm;
                        }
                    }

                    if (before_pb_retracted_m[num] < 0.0f) before_pb_retracted_m[num] = 0.0f;
                    if (before_pb_retracted_m[num] > 2.0f) before_pb_retracted_m[num] = 2.0f;
                }

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_before_pull_back, 300, time_now);
                break;
            }

            case _filament_motion::on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_on_use, 300, time_now);
                MC_STU_RGB_set_latch(num, 0x00u, 0xB0u, 0xFFu, time_now, 0u);
                break;
            }

            case _filament_motion::idle:
            default:
            {
                filament_now_position[num] = filament_idle;

                if (g_on_use_jam_latch[num])
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
                    MC_STU_RGB_set_latch(num, STU_OK_R, STU_OK_G, STU_OK_B, time_now, 0u);
                    break;
                }

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100, time_now);

#if BMCU_DM_TWO_MICROSWITCH
                if (dm_fail_latch[num])      MC_STU_RGB_set_latch(num, 0xFFu, 0x00u, 0x00u, time_now, 0u);
                else if (dm_loaded[num])     MC_STU_RGB_set_latch(num, STU_OK_R, STU_OK_G, STU_OK_B, time_now, 0u);
                else                         MC_STU_RGB_set_latch(num, 0x00u, 0x00u, 0x00u, time_now, 0u);
#else
                MC_STU_RGB_set_latch(num, STU_OK_R, STU_OK_G, STU_OK_B, time_now, 0u);
#endif
                break;
            }
            }
        }
        else
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100, time_now);
            MC_STU_RGB_set_latch(num, 0x00u, 0x00u, 0x00u, time_now, 0u);
        }
    }
}

static inline void stu_apply_baseline(int error, uint64_t now_ms)
{
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (g_on_use_low_latch[i])
        {
            MC_STU_RGB_set(i, 0xFFu, 0x00u, 0x00u);
            continue;
        }

#if BMCU_DM_TWO_MICROSWITCH
        if (dm_fail_latch[i])
        {
            MC_STU_RGB_set(i, 0xFFu, 0x00u, 0x00u);
            continue;
        }

        const bool ins_ok = error ? true : filament_channel_inserted[i];
        const bool show_loaded =
            (dm_loaded[i] != 0u) &&
            (MC_ONLINE_key_stu[i] != 0u) &&
            ins_ok;

        if (show_loaded) MC_STU_RGB_set_latch(i, STU_OK_R, STU_OK_G, STU_OK_B, now_ms, 0u);
        else             MC_STU_RGB_set_latch(i, 0x00u, 0x00u, 0x00u, now_ms, 0u);
#else
        if (error)
        {
            if (MC_ONLINE_key_stu[i] != 0) MC_STU_RGB_set_latch(i, STU_OK_R, STU_OK_G, STU_OK_B, now_ms, 0u);
            else                           MC_STU_RGB_set_latch(i, 0x00u, 0x00u, 0x00u, now_ms, 0u);
        }
        else
        {
            if (MC_ONLINE_key_stu[i] != 0 && filament_channel_inserted[i])
                MC_STU_RGB_set_latch(i, STU_OK_R, STU_OK_G, STU_OK_B, now_ms, 0u);
            else
                MC_STU_RGB_set_latch(i, 0x00u, 0x00u, 0x00u, now_ms, 0u);
        }
#endif
    }
}


static void motor_motion_run(int error, uint64_t time_now, uint32_t now_ticks)
{
#if BMCU_DM_TWO_MICROSWITCH
    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            dm_loaded[ch]            = 1u;
            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_m[ch]     = 0.0f;
            dm_auto_last_m[ch]       = 0.0f;
            dm_loaded_drop_t0_ms[ch] = 0ull;
            dm_autoload_gate[ch]     = 0u;
            continue;
        }

        const uint8_t ks = MC_ONLINE_key_stu[ch];

        if (ks == 0u)
        {
            if (filament_now_position[ch] == filament_idle)
                dm_autoload_gate[ch] = 0u;

            dm_loaded[ch]            = 0u;
            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_m[ch]     = 0.0f;
            dm_auto_last_m[ch]       = 0.0f;
            dm_loaded_drop_t0_ms[ch] = 0ull;
            continue;
        }

        if (dm_loaded[ch] && (ks != 1u))
        {
            uint64_t t0 = dm_loaded_drop_t0_ms[ch];
            if (t0 == 0ull) dm_loaded_drop_t0_ms[ch] = time_now;
            else if ((time_now - t0) >= 100ull)
            {
                dm_loaded[ch]            = 0u;
                dm_loaded_drop_t0_ms[ch] = 0ull;

                dm_auto_state[ch]    = DM_AUTO_IDLE;
                dm_auto_try[ch]      = 0u;
                dm_auto_t0_ms[ch]    = 0ull;
                dm_auto_remain_m[ch] = 0.0f;
                dm_auto_last_m[ch]   = 0.0f;
            }
        }
        else
        {
            dm_loaded_drop_t0_ms[ch] = 0ull;
        }
    }
#endif

    static uint32_t last_ticks = 0u;
    static uint8_t  have_last_ticks = 0u;

    uint32_t dt_ticks = 0u;
    if (!have_last_ticks)
    {
        have_last_ticks = 1u;
    }
    else
    {
        dt_ticks = (uint32_t)(now_ticks - last_ticks);
    }
    last_ticks = now_ticks;

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    const uint32_t max_dt_ticks = 200u * tpm;
    if (dt_ticks > max_dt_ticks) dt_ticks = max_dt_ticks;

    uint32_t tpus = time_hw_tpus;
    if (!tpus) tpus = 1u;

    const bool have_time_step = (dt_ticks != 0u);
    const float time_E = have_time_step ? ((float)dt_ticks / ((float)tpus * 1000000.0f)) : 0.0f;

    stu_apply_baseline(error, time_now);

#if BMCU_ONLINE_LED_FILAMENT_RGB
    auto &Acol = ams[motion_control_ams_num];
#endif

    if (!error)
    {
        if (!motor_motion_filamnet_pull_back_to_online_key(time_now))
            motor_motion_switch(time_now);
    }
    else
    {
        for (uint8_t i = 0; i < kChCount; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
    }

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (!AS5600_is_good(i))
        {
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100, time_now);
            Motion_control_set_PWM(i, 0);
            continue;
        }

        if (!filament_channel_inserted[i] ||
            (!auto_unload_active[i] && MOTOR_CONTROL[i].motion != filament_motion_enum::filament_motion_pressure_ctrl_idle))
        {
            auto_unload_arm[i]          = 0u;
            auto_unload_active[i]       = 0u;
            auto_unload_blocked[i]      = 0u;
            auto_unload_arm_t0_ms[i]    = 0ull;
            auto_unload_active_t0_ms[i] = 0ull;
            auto_unload_empty_t0_ms[i]  = 0ull;
        }
        else
        {
            const float pct = MC_PULL_pct_f[i];
            const uint8_t ks = MC_ONLINE_key_stu[i];

            if (pct >= AUTO_UNLOAD_START_PCT)
            {
                auto_unload_blocked[i] = 0u;

                if (!auto_unload_arm[i] && !auto_unload_active[i])
                {
                    auto_unload_arm[i] = 1u;
                    auto_unload_arm_t0_ms[i] = time_now;
                }
            }

            if (auto_unload_arm[i] && !auto_unload_active[i])
            {
                const uint64_t dt = time_now - auto_unload_arm_t0_ms[i];

                if ((pct > AUTO_UNLOAD_NEUTRAL_LO_PCT) && (pct < AUTO_UNLOAD_NEUTRAL_HI_PCT))
                {
                    if (!auto_unload_blocked[i] && dt <= AUTO_UNLOAD_ARM_MS)
                    {
                        auto_unload_active[i]       = 1u;
                        auto_unload_active_t0_ms[i] = time_now;
                        auto_unload_empty_t0_ms[i]  = 0ull;
                        auto_unload_blocked[i]      = 1u;
                    }

                    auto_unload_arm[i]       = 0u;
                    auto_unload_arm_t0_ms[i] = 0ull;
                }
                else if (dt > AUTO_UNLOAD_ARM_MS)
                {
                    auto_unload_arm[i]       = 0u;
                    auto_unload_arm_t0_ms[i] = 0ull;
                }
            }

            if (auto_unload_active[i])
            {
                if (pct < AUTO_UNLOAD_ABORT_PCT)
                {
                    auto_unload_active[i]       = 0u;
                    auto_unload_active_t0_ms[i] = 0ull;
                    auto_unload_empty_t0_ms[i]  = 0ull;
                    auto_unload_blocked[i]      = 1u;
                }
                else if (ks == 1u)
                {
                    auto_unload_empty_t0_ms[i] = 0ull;

                    if ((time_now - auto_unload_active_t0_ms[i]) >= AUTO_UNLOAD_MAX_MS)
                    {
                        auto_unload_active[i]       = 0u;
                        auto_unload_active_t0_ms[i] = 0ull;
                        auto_unload_empty_t0_ms[i]  = 0ull;
                        auto_unload_blocked[i]      = 1u;
                    }
                }
                else
                {
                    if (auto_unload_empty_t0_ms[i] == 0ull)
                    {
                        auto_unload_empty_t0_ms[i] = time_now;
                    }
                    else if ((time_now - auto_unload_empty_t0_ms[i]) >= AUTO_UNLOAD_EMPTY_MS)
                    {
                        auto_unload_active[i]       = 0u;
                        auto_unload_active_t0_ms[i] = 0ull;
                        auto_unload_empty_t0_ms[i]  = 0ull;
                        auto_unload_blocked[i]      = 1u;
                    }
                }
            }
        }

        const bool manual_empty_pull =
            filament_channel_inserted[i] &&
            (MC_ONLINE_key_stu[i] == 0u) &&
            (MC_PULL_pct_f[i] > 80.0f) &&
            (auto_unload_active[i] == 0u);

        if (auto_unload_active[i])
        {
            float x = MOTOR_CONTROL[i].dir * AUTO_UNLOAD_PWM_PULL;
            if (x * MOTOR_CONTROL[i].dir < 0.0f) x = 0.0f;

            MOTOR_CONTROL[i].PID_speed.clear();
            MOTOR_CONTROL[i].PID_pressure.clear();
            MOTOR_CONTROL[i].pwm_zeroed = (x == 0.0f) ? 1u : 0u;
            _MOTOR_CONTROL::x_prev[i] = x;

            Motion_control_set_PWM(i, (int)x);
            MC_STU_RGB_set_latch(i, 0xA0u, 0x2Du, 0xFFu, time_now, 1u);
        }
        else if (manual_empty_pull)
        {
            float x = MOTOR_CONTROL[i].dir * 700.0f;
            if (x * MOTOR_CONTROL[i].dir < 0.0f) x = 0.0f;

            MOTOR_CONTROL[i].PID_speed.clear();
            MOTOR_CONTROL[i].PID_pressure.clear();
            MOTOR_CONTROL[i].pwm_zeroed = (x == 0.0f) ? 1u : 0u;
            _MOTOR_CONTROL::x_prev[i] = x;

            Motion_control_set_PWM(i, (int)x);
        }
        else if (have_time_step)
        {
            MOTOR_CONTROL[i].run(time_E, time_now);
        }

        uint8_t r = 0u, g = 0u, b = 0u;
        bool is_filament_rgb = false;

        const uint8_t pct = MC_PULL_pct[i];

        int hi_thr = MC_PULL_DEADBAND_PCT_HIGH;

        const filament_motion_enum m = MOTOR_CONTROL[i].motion;

        bool hi_hold =
            (m == filament_motion_enum::filament_motion_send) ||
            (m == filament_motion_enum::filament_motion_before_on_use) ||
            (m == filament_motion_enum::filament_motion_stop_on_use);

        if (!hi_hold && (m == filament_motion_enum::filament_motion_pressure_ctrl_on_use))
        {
            const uint64_t t0 = MOTOR_CONTROL[i].on_use_hi_gate_t0_ms;
            if (t0 != 0ull && (time_now - t0) < 5000ull) hi_hold = true;
        }

        if (hi_hold)
        {
            hi_thr = (int)MC_LOAD_S2_HOLD_TARGET_PCT + 3;
            if (hi_thr > 100) hi_thr = 100;
            if (hi_thr < 0) hi_thr = 0;
        }

        if (!(m == filament_motion_enum::filament_motion_before_on_use) && (int)pct >= hi_thr)
        {
            r = 0x10u;
        }
        else if (pct <= 30u)
        {
            b = 0x10u;
        }
        else
        {
            const uint8_t key = MC_ONLINE_key_stu[i];

#if BMCU_ONLINE_LED_FILAMENT_RGB
    #if BMCU_DM_TWO_MICROSWITCH
            const bool show_filament_rgb = (key == 1u) && dm_loaded[i] && !dm_fail_latch[i];
    #else
            const bool show_filament_rgb = (key != 0u);
    #endif
            if (show_filament_rgb)
            {
                r = Acol.filament[i].color_R;
                g = Acol.filament[i].color_G;
                b = Acol.filament[i].color_B;
                is_filament_rgb = true;
            }
            else
#endif
            {
                if (key == 0u)
                {
                    if ((uint8_t)(pct - 49u) <= 2u) { r = 0x10u; g = 0x08u; }
                }
            }
        }

        MC_PULL_ONLINE_RGB_set(i, r, g, b, is_filament_rgb);
    }
}

void Motion_control_run(int error)
{
    const uint64_t now_ticks64 = time_ticks64();
    const uint32_t now_ticks   = (uint32_t)now_ticks64;
    const uint64_t now_ms      = time_ms_fast_from_ticks64(now_ticks64);

    MC_PULL_ONLINE_read(now_ticks);

    const uint8_t loaded_ch = ams_state_get_loaded();
    if ((loaded_ch < kChCount) && (MC_ONLINE_key_stu[loaded_ch] == 0u))
        ams_state_set_unloaded(loaded_ch);

    auto &A = ams[motion_control_ams_num];

    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        const uint8_t ks = MC_ONLINE_key_stu[ch];
        if (ks == 0u)
        {
            if (!error)
            {
                if (A.now_filament_num == ch)
                {
                    if (A.filament[ch].motion == _filament_motion::send_out)
                        MOTOR_CONTROL[ch].set_motion(filament_motion_enum::filament_motion_stop, 100, now_ms);
                }
            }

            if (g_on_use_jam_latch[ch])
            {
                g_on_use_low_latch[ch] = 0u;
                g_on_use_jam_latch[ch] = 0u;
            }

            g_on_use_hi_pwm_us[ch] = 0u;
        }
    }

    if (!error)
    {
        const uint8_t n = A.now_filament_num;

        if ((n < kChCount) && filament_channel_inserted[n] && g_on_use_jam_latch[n])
        {
            const _filament_motion m = A.filament[n].motion;

            if (m == _filament_motion::on_use || m == _filament_motion::send_out)
                A.pressure = 0xF06Fu;
        }
    }

    if ((error <= 0) && all_no_filament())
    {
        int pressed = -1;

        for (uint8_t ch = 0; ch < kChCount; ch++)
        {
            if (!filament_channel_inserted[ch]) continue;

            const int   pct = (int)MC_PULL_pct[ch];
            const float v   = MC_PULL_stu_raw[ch];

            const bool hard_blue =
                (pct <= CAL_RESET_PCT_THRESH) ||
                (v <= (1.65f - CAL_RESET_V_DELTA)) ||
                (v <= (MC_PULL_V_MIN[ch] + CAL_RESET_NEAR_MIN));

            if (hard_blue) { pressed = (int)ch; break; }
        }

        uint32_t tpm = time_hw_tpms;
        if (!tpm) tpm = 1u;

        if (pressed >= 0)
        {
            if (g_hold_ch != pressed)
            {
                g_hold_ch = pressed;
                g_hold_t0_ticks = now_ticks;
            }
            else
            {
                if ((uint32_t)(now_ticks - g_hold_t0_ticks) >= (uint32_t)CAL_RESET_HOLD_MS * tpm)
                    calibration_reset_and_reboot();
            }
        }
        else
        {
            g_hold_ch = -1;
            g_hold_t0_ticks = 0u;
        }
    }
    else
    {
        g_hold_ch = -1;
        g_hold_t0_ticks = 0u;
    }

    AS5600_distance_updata(now_ticks);

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (MC_ONLINE_key_stu[i] != 0u) A.filament[i].online = true;
        else if ((filament_now_position[i] == filament_redetect) || (filament_now_position[i] == filament_pulling_back))
            A.filament[i].online = true;
        else
            A.filament[i].online = false;
    }

    motor_motion_run(error, now_ms, now_ticks);

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1))
            MC_STU_RGB_set(i, 0xFF, 0x00, 0x00);
    }
}

// ===== PWM init =====
void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                    GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Period        = 999;
    TIM_TimeBaseStructure.TIM_Prescaler     = 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// różnica kątów
static inline int M5600_angle_dis(int16_t angle1, int16_t angle2)
{
    int d = (int)angle1 - (int)angle2;
    if (d >  2048) d -= 4096;
    if (d < -2048) d += 4096;
    return d;
}

// test kierunku silników
static void MOTOR_get_dir()
{
    int  dir[4]     = {0,0,0,0};
    bool test[4]    = {false,false,false,false};
    bool any_detect = false;
    bool any_change = false;
    bool timed_out  = false;

    const bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (uint8_t i = 0; i < kChCount; i++)
            Motion_control_data_save.Motion_control_dir[i] = 0;
    }

    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (uint8_t i = 0; i < kChCount; i++)
    {
        last_angle[i] = MC_AS5600.raw_angle[i];
        dir[i] = Motion_control_data_save.Motion_control_dir[i];
    }

    // Start test tylko tam, gdzie:
    // - AS5600 online
    // - kanał fizycznie wpięty
    // - dir nieznany (0)
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (AS5600_is_good(i) && filament_channel_inserted[i] && (dir[i] == 0))
        {
            Motion_control_set_PWM(i, 1000);
            test[i] = true;
        }
    }

    // jeśli nie ma nic do testowania -> nie rób NIC, nie zapisuj, nie psuj
    if (!(test[0] || test[1] || test[2] || test[3]))
        return;

    // czekaj max 2s na ruch (200 * 10ms)
    for (int t = 0; t < 200; t++)
    {
        delay(10);
        MC_AS5600.updata_angle();

        bool done = true;

        for (uint8_t i = 0; i < kChCount; i++)
        {
            if (!test[i]) continue;

            // jeśli czujnik zniknął po drodze -> abort kanału (nie zapisuj)
            if (!MC_AS5600.online[i])
            {
                Motion_control_set_PWM(i, 0);
                test[i] = false;
                continue;
            }

            const int angle_dis = M5600_angle_dis((int16_t)MC_AS5600.raw_angle[i], last_angle[i]);

            if ((angle_dis > 163) || (angle_dis < -163))
            {
                Motion_control_set_PWM(i, 0);

                // AS5600 odwrotnie względem magnesu
                dir[i] = (angle_dis > 0) ? 1 : -1;

                test[i] = false;
                any_detect = true;
            }
            else
            {
                done = false;
            }
        }

        if (done) break;
        if (t == 199) timed_out = true;
    }

    // stop dla niedokończonych
    for (uint8_t i = 0; i < kChCount; i++)
        if (test[i]) Motion_control_set_PWM(i, 0);

    // zaktualizuj tylko tam, gdzie faktycznie zmieniło się dir
    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (dir[i] != Motion_control_data_save.Motion_control_dir[i])
        {
            Motion_control_data_save.Motion_control_dir[i] = dir[i];
            any_change = true;
        }
    }

    // zapis tylko jeśli była realna detekcja ruchu (dir => ±1)
    // Jak brak 24V i nic się nie ruszyło -> any_detect=false -> NIE zapisujemy.
    if (any_detect && any_change)
    {
        Motion_control_save();
    }
    else
    {
        (void)timed_out;
    }
}

// init motorów
static void MOTOR_init()
{
    MC_PWM_init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    MOTOR_get_dir();

    for (uint8_t i = 0; i < kChCount; i++)
    {
        Motion_control_set_PWM(i, 0);
        MOTOR_CONTROL[i].set_pwm_zero(500);
        MOTOR_CONTROL[i].dir = (float)Motion_control_data_save.Motion_control_dir[i];
    }
}

void Motion_control_init()
{
    auto &A = ams[motion_control_ams_num];
    A.online   = true;
    A.ams_type = 0x03;

    (void)Motion_control_read();

    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read(time_ticks32());

    #if BMCU_DM_TWO_MICROSWITCH
        for (uint8_t ch = 0; ch < kChCount; ch++)
        {
            if (!filament_channel_inserted[ch])
            {
                dm_loaded[ch]            = 1u;
                dm_fail_latch[ch]        = 0u;
                dm_auto_state[ch]        = DM_AUTO_IDLE;
                dm_auto_try[ch]          = 0u;
                dm_auto_t0_ms[ch]        = 0ull;
                dm_auto_remain_m[ch]     = 0.0f;
                dm_auto_last_m[ch]       = 0.0f;
                dm_loaded_drop_t0_ms[ch] = 0ull;
                dm_autoload_gate[ch]     = 0u;
                continue;
            }

            const uint8_t ks = MC_ONLINE_key_stu[ch];

            dm_autoload_gate[ch] = (ks != 0u) ? 1u : 0u;
            dm_loaded[ch] = (ks == 1u) ? 1u : 0u;

            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_m[ch]     = 0.0f;
            dm_auto_last_m[ch]       = 0.0f;
            dm_loaded_drop_t0_ms[ch] = 0ull;
        }
    #endif

    MC_AS5600.init(AS5600_SCL_PORT, AS5600_SCL_PIN,
               AS5600_SDA_PORT, AS5600_SDA_PIN,
               4);
    MC_AS5600.updata_angle();
    MC_AS5600.updata_stu();

    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ok = MC_AS5600.online[i] && (MC_AS5600.magnet_stu[i] != AS5600_soft_IIC_many::offline);
        g_as5600_good[i]     = ok ? 1u : 0u;
        g_as5600_fail[i]     = ok ? 0u : kAS5600_FAIL_TRIP;
        g_as5600_okstreak[i] = ok ? kAS5600_OK_RECOVER : 0u;
    }

    for (uint8_t i = 0; i < kChCount; i++)
    {
        as5600_distance_save[i] = MC_AS5600.raw_angle[i];
        filament_now_position[i] = filament_idle;
    }

    MOTOR_init();
}
