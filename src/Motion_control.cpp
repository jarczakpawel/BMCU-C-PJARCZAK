#include "Motion_control.h"
#include "ams.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "_bus_hardware.h"
#include "many_soft_AS5600.h"
#include "app_api.h"
#include "hal/time_hw.h"
#include "../CONFIG_DOUBLE_MICROSWITCH.h"

static inline float absf(float x) { return (x < 0.0f) ? -x : x; }
static inline float clampf(float x, float a, float b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
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
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        mag = 450.0f + 100.0f * t;
    }
    else
    {
        float t = (err - e1) / (e2 - e1);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
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
static constexpr float PULL_V_FAST   = 50.0f;   // mm/s
static constexpr float PULL_V_END    = 12.0f;   // mm/s na samym końcu
static constexpr float PULL_RAMP_M   = 0.015f;  // 15mm strefa hamowania
static constexpr float PULL_PWM_MIN  = 400.0f;  // "kop" przy pullback

static float g_pull_remain_m[4]  = {0,0,0,0};
static float g_pull_speed_set[4] = {-PULL_V_FAST,-PULL_V_FAST,-PULL_V_FAST,-PULL_V_FAST}; // mm/s (ujemne)

float MC_PULL_V_OFFSET[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float MC_PULL_V_MIN[4]    = {1.00f, 1.00f, 1.00f, 1.00f};
float MC_PULL_V_MAX[4]    = {2.00f, 2.00f, 2.00f, 2.00f};

uint8_t MC_PULL_pct[4]    = {50, 50, 50, 50};
static float MC_PULL_pct_f[4] = {50.0f, 50.0f, 50.0f, 50.0f};

static float  MC_PULL_stu_raw[4]        = {1.65f, 1.65f, 1.65f, 1.65f};
static int8_t MC_PULL_stu[4]            = {0, 0, 0, 0};

static uint8_t MC_ONLINE_key_stu[4]     = {0, 0, 0, 0};
static float   MC_ONLINE_key_stu_raw[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Raw voltage for dual microswitch

bool filament_channel_inserted[4]       = {false, false, false, false}; // czy kanał fizycznie wpięty

static constexpr float MC_PULL_PIDP_PCT = 25.0f;

static constexpr int MC_PULL_DEADBAND_PCT_LOW  = 30;
static constexpr int MC_PULL_DEADBAND_PCT_HIGH = 70;

static constexpr int MC_PULL_SEND_FAST_PCT     = 53;
static constexpr int MC_PULL_SEND_STOP_PCT     = 56;
static constexpr int MC_PULL_SEND_HARD_STOP_PCT = 70;  // bezpiecznik
static constexpr int MC_PULL_SEND_HARD_HYS      = 2;   // wróć dopiero < (HARD_STOP - HYS)

static constexpr uint32_t CAL_RESET_HOLD_MS     = 5000;
static constexpr int      CAL_RESET_PCT_THRESH  = 15;
static constexpr float    CAL_RESET_V_DELTA     = 0.10f;
static constexpr float    CAL_RESET_NEAR_MIN    = 0.03f;

static int      g_hold_ch = -1;
static uint32_t g_hold_t0_ticks = 0;

// kiedy kanał OSTATNIO wyszedł z on_use (0 = nigdy, 1 = marker "był kiedykolwiek") (patch do wersji BMCU DM przy automatycznej zmianie filamentu gdy się skończy, żeby ekstruder nie trzymał filamentu)
static uint64_t g_last_on_use_exit_ms[4] = {0,0,0,0};

extern void RGB_update();
extern bool Flash_MC_PULL_cal_clear();
static inline void Motion_control_dir_clear_and_save();

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
    // stop wszystko
    for (uint8_t i = 0; i < kChCount; i++) Motion_control_set_PWM(i, 0);

    blink_all_blue_3s();
    // kasuj kalibrację PULL
    Flash_MC_PULL_cal_clear();

    // kasuj zapisany kierunek silników -> wymusi MOTOR_get_dir() po boocie
    Motion_control_dir_clear_and_save();

    // reboot
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
        float den = (c - vmin);
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f * (v - vmin) / den;
    }
    else
    {
        float den = (vmax - c);
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f + 0.5f * (v - c) / den;
    }

    pos01 = clampf(pos01, 0.0f, 1.0f);
    return pos01 * 100.0f;
}

void MC_PULL_detect_channels_inserted()
{
    if (!ADC_DMA_is_inited())
    {
        for (int ch = 0; ch < 4; ch++) filament_channel_inserted[ch] = false;
        return;
    }

    ADC_DMA_gpio_analog();
    ADC_DMA_filter_reset();
    (void)ADC_DMA_wait_full();

    constexpr int N = 16;
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0;

    for (int i = 0; i < N; i++)
    {
        const float *v = ADC_DMA_get_value();

        s0 += v[6];
        s1 += v[4];
        s2 += v[2];
        s3 += v[0];

        delay(2);
    }

    const float a0 = s0 / (float)N;
    const float a1 = s1 / (float)N;
    const float a2 = s2 / (float)N;
    const float a3 = s3 / (float)N;

    constexpr float VMIN = 0.30f;
    constexpr float VMAX = 3.00f;

    filament_channel_inserted[0] = (a0 > VMIN) && (a0 < VMAX);
    filament_channel_inserted[1] = (a1 > VMIN) && (a1 < VMAX);
    filament_channel_inserted[2] = (a2 > VMIN) && (a2 < VMAX);
    filament_channel_inserted[3] = (a3 > VMIN) && (a3 < VMAX);
}



static inline void MC_PULL_ONLINE_init()
{
    MC_PULL_detect_channels_inserted();
}

static inline void MC_PULL_ONLINE_read()
{
    const float *data = ADC_DMA_get_value();

    // mapowanie ADC -> kanały
    MC_PULL_stu_raw[3] = data[0] + MC_PULL_V_OFFSET[3];
    const float key3   = data[1];

    MC_PULL_stu_raw[2] = data[2] + MC_PULL_V_OFFSET[2];
    const float key2   = data[3];

    MC_PULL_stu_raw[1] = data[4] + MC_PULL_V_OFFSET[1];
    const float key1   = data[5];

    MC_PULL_stu_raw[0] = data[6] + MC_PULL_V_OFFSET[0];
    const float key0   = data[7];

    // Save raw voltages for dual microswitch diagnostics
    MC_ONLINE_key_stu_raw[3] = key3;
    MC_ONLINE_key_stu_raw[2] = key2;
    MC_ONLINE_key_stu_raw[1] = key1;
    MC_ONLINE_key_stu_raw[0] = key0;

    // ===== DUAL MICROSWITCH INTERPRETATION =====
    // MC_ONLINE_key_stu[i] states:
    // 0 = Offline - no filament (V < 0.6V)
    // 1 = Online - both microswitches activated (V > 1.7V)
    // 2 = External only - needs assistance (1.4V < V < 1.7V)
    // 3 = Internal only - anomaly situation (0.6V < V < 1.4V)
    
    if (IS_TWO_MICROSWITCH_ENABLED)
    {
        // DUAL MICROSWITCH system active
        for (int i = 0; i < 4; i++)
        {
            if (!filament_channel_inserted[i])
            {
                MC_ONLINE_key_stu[i] = 0;
                continue;
            }

            const float key_voltage = MC_ONLINE_key_stu_raw[i];

            if (key_voltage < THRESHOLD_OFFLINE)
            {
                // No microswitch pressed - filament offline
                MC_ONLINE_key_stu[i] = 0;
            }
            else if ((key_voltage > THRESHOLD_EXTERNAL_MIN) && (key_voltage < THRESHOLD_EXTERNAL_MAX))
            {
                // Only EXTERNAL microswitch pressed - NEEDS ASSISTANCE
                MC_ONLINE_key_stu[i] = 2;
            }
            else if (key_voltage > THRESHOLD_BOTH)
            {
                // BOTH microswitches pressed - filament fully inserted
                MC_ONLINE_key_stu[i] = 1;
            }
            else // (key_voltage >= THRESHOLD_OFFLINE && key_voltage <= THRESHOLD_EXTERNAL_MIN)
            {
                // Only INTERNAL microswitch pressed - anomaly situation
                MC_ONLINE_key_stu[i] = 3;
            }
        }
    }
    else
    {
        // SINGLE MICROSWITCH system (classic mode)
        MC_ONLINE_key_stu[3] = (filament_channel_inserted[3] && (key3 > 1.65f)) ? 1u : 0u;
        MC_ONLINE_key_stu[2] = (filament_channel_inserted[2] && (key2 > 1.65f)) ? 1u : 0u;
        MC_ONLINE_key_stu[1] = (filament_channel_inserted[1] && (key1 > 1.65f)) ? 1u : 0u;
        MC_ONLINE_key_stu[0] = (filament_channel_inserted[0] && (key0 > 1.65f)) ? 1u : 0u;
    }


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
        int pct = (int)MC_PULL_pct[num];
        int hi  = pct - 50;
        if (hi < 0) hi = 0;
        if (hi > 50) hi = 50;

        const uint32_t pressure = (uint32_t)((hi * 65535u) / 50u);
        A.pressure = (int)pressure;
    }
    else
    {
        A.pressure = 0xFFFF;
    }
}

// ===== zapis kierunku silników =====
struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    uint32_t check = 0x40614061u;
} Motion_control_data_save;

#define Motion_control_save_flash_addr FLASH_NVM_MOTION_ADDR

static inline bool Motion_control_read()
{
    if (!Flash_Motion_read(&Motion_control_data_save, (uint16_t)sizeof(Motion_control_save_struct)))
        return false;

    if (Motion_control_data_save.check != 0x40614061u)
    {
        for (uint8_t i = 0; i < kChCount; i++) Motion_control_data_save.Motion_control_dir[i] = 0;
        Motion_control_data_save.check = 0x40614061u;
        return false;
    }
    return true;
}

static inline void Motion_control_save()
{
    (void)Flash_Motion_write(&Motion_control_data_save, (uint16_t)sizeof(Motion_control_save_struct));
}

static inline void Motion_control_dir_clear_and_save()
{
    for (uint8_t i = 0; i < kChCount; i++)
        Motion_control_data_save.Motion_control_dir[i] = 0;

    Motion_control_data_save.check = 0x40614061u;
    Motion_control_save();
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
    uint64_t post_sendout_until_ms = 0;
    uint8_t  retract_hys_active = 0;

    uint64_t send_start_ms = 0;
    uint64_t pull_start_ms = 0;

    uint64_t send_stop_brake_ms = 0;

    bool send_stop_latch = false;

    MOTOR_PID PID_speed    = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(MC_PULL_PIDP_PCT, 0, 0);

    float pwm_zero = 500;
    float dir = 0;

    static float x_prev[4];

    bool  send_hold = false;
    bool  send_hard = false;
    float send_base = 0.0f;
    float send_pct_prev = 50.0f;

    _MOTOR_CONTROL(int _CHx) : CHx(_CHx) {}

    void set_pwm_zero(float _pwm_zero) { pwm_zero = _pwm_zero; }

    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        set_motion(_motion, over_time, get_time64());
    }

    void set_motion(filament_motion_enum _motion, uint64_t over_time, uint64_t time_now)
    {
        motor_stop_time = (_motion == filament_motion_enum::filament_motion_stop) ? 0 : (time_now + over_time);

        if (motion == _motion) return;

        const filament_motion_enum prev = motion;
        motion = _motion;
        pwm_zeroed = 0;

        if (_motion == filament_motion_enum::filament_motion_send) {
            send_start_ms = time_now;
            send_stop_latch = false;
        }

        if (_motion == filament_motion_enum::filament_motion_pull) {
            pull_start_ms = time_now;
        }

        if (prev == filament_motion_enum::filament_motion_send &&
            _motion != filament_motion_enum::filament_motion_send)
        {
            send_start_ms = 0;
            send_stop_latch = false;
        }

        if (prev == filament_motion_enum::filament_motion_pull &&
            _motion != filament_motion_enum::filament_motion_pull)
        {
            pull_start_ms = 0;
        }

        // marker "ever on_use"
        if (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            if (g_last_on_use_exit_ms[CHx] == 0) g_last_on_use_exit_ms[CHx] = 1;
        }

        // exit timestamp
        if (prev == filament_motion_enum::filament_motion_pressure_ctrl_on_use &&
            _motion != filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            g_last_on_use_exit_ms[CHx] = time_now;
        }

        // CLEAR
        if (_motion == filament_motion_enum::filament_motion_send ||
            _motion == filament_motion_enum::filament_motion_pull)
        {
            g_last_on_use_exit_ms[CHx] = 0;
        }

        // SEND reset
        if (_motion == filament_motion_enum::filament_motion_send)
        {
            send_hold = false; send_hard = false; send_base = 0.0f;
            send_pct_prev = MC_PULL_pct_f[CHx];
        }

        if (prev == filament_motion_enum::filament_motion_send &&
            _motion != filament_motion_enum::filament_motion_send)
        {
            send_hold = false; send_hard = false; send_base = 0.0f;
        }

        PID_speed.clear();
        PID_pressure.clear();

        const bool keep_pwm =
            (prev == filament_motion_enum::filament_motion_send) &&
            (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use);

        if (_motion == filament_motion_enum::filament_motion_send)
        {
            post_sendout_retract_thresh_pct = -1.0f;
            post_sendout_until_ms = 0;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_before_on_use)
        {
            float p = MC_PULL_pct_f[CHx];
            if (p < 0.0f) p = 0.0f;
            if (p > 100.0f) p = 100.0f;
            post_sendout_retract_thresh_pct = p;
            post_sendout_until_ms = 0;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            if (post_sendout_retract_thresh_pct >= 0.0f)
                post_sendout_until_ms = time_now + 10000ull;
            else
                post_sendout_until_ms = 0;

            retract_hys_active = 0;
        }
        else if (prev == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
        {
            post_sendout_until_ms = 0;
            post_sendout_retract_thresh_pct = -1.0f;
            retract_hys_active = 0;
        }

        if (_motion == filament_motion_enum::filament_motion_pull)
        {
            post_sendout_until_ms = 0;
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

    void run(float time_E, uint64_t now_ms)
    {
        if (motion == filament_motion_enum::filament_motion_stop &&
            motor_stop_time == 0 &&
            pwm_zeroed)
            return;

        const bool is_on_use        = (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use);

        // auto-clear po 10s w ON_USE
        if (is_on_use && post_sendout_until_ms && now_ms >= post_sendout_until_ms)
        {
            post_sendout_until_ms = 0;
            post_sendout_retract_thresh_pct = -1.0f;
            retract_hys_active = 0;
        }

        const bool post_sendout_10s =
            is_on_use &&
            (post_sendout_until_ms != 0) &&
            (now_ms < post_sendout_until_ms) &&
            (post_sendout_retract_thresh_pct >= 0.0f);

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

        float speed_set = 0.0f;
        const float now_speed = speed_as5600[CHx];
        float x = 0.0f;

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
            post_on_use_10s;

        bool  on_use_need_move = false;
        float on_use_abs_err   = 0.0f;

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)
        {
            if (MC_ONLINE_key_stu[CHx] == 0)
            {
                // Normal handling when assistance disabled or single microswitch system
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
                        const float err = pct - 49.0f;   // ujemny => pchanie

                        on_use_need_move = true;
                        on_use_abs_err   = (err < 0.0f) ? -err : err;

                        x = dir * PID_pressure.caculate(err, time_E);

                        float lim_f = 500.0f + 80.0f * on_use_abs_err;
                        if (lim_f > 900.0f) lim_f = 900.0f;

                        if (x >  lim_f) x =  lim_f;
                        if (x < -lim_f) x = -lim_f;

                        // tylko pchanie (zakaz cofania)
                        if (x * dir > 0.0f)
                        {
                            x = 0.0f;
                            PID_pressure.clear();
                            on_use_need_move = false;
                            on_use_abs_err = 0.0f;
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

                constexpr float hold_target = 52.0f;
                constexpr float band_lo     = 51.7f;

                float thresh = post_sendout_retract_thresh_pct;
                if (thresh < hold_target) thresh = hold_target;

                // RETRACT tylko gdy pct > thresh (jak before_pull_back)
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
                    }
                    else
                    {
                        const float err = pct - target; // dodatni
                        on_use_need_move = true;
                        on_use_abs_err   = err;

                        const float mag = retract_mag_from_err(err, 850.0f);

                        x = dir * mag; // tylko cofanie
                        if (x * dir < 0.0f) x = 0.0f;
                    }
                }
                else
                {
                    retract_hys_active = 0;

                    // PUSH-only PID do 52%
                    if (pct >= band_lo)
                    {
                        x = 0.0f;
                        PID_pressure.clear();
                        on_use_need_move = false;
                        on_use_abs_err   = 0.0f;
                    }
                    else
                    {
                        const float err = pct - hold_target; // ujemny => pchanie
                        on_use_need_move = true;
                        on_use_abs_err   = -err;

                        x = dir * PID_pressure.caculate(err, time_E);

                        float lim_f = 550.0f + 90.0f * on_use_abs_err;
                        if (lim_f > 950.0f) lim_f = 950.0f;

                        if (x >  lim_f) x =  lim_f;
                        if (x < -lim_f) x = -lim_f;

                        // zakaz cofania
                        if (x * dir > 0.0f)
                        {
                            x = 0.0f;
                            PID_pressure.clear();
                            on_use_need_move = false;
                            on_use_abs_err   = 0.0f;
                        }
                    }
                }
            }
            else if (motion == filament_motion_enum::filament_motion_stop_on_use)
            {
                const float pct = MC_PULL_pct_f[CHx];

                float thresh = post_sendout_retract_thresh_pct;
                if (thresh < 0.0f) thresh = 1000.0f; // invalid => nigdy nie cofnij

                if (pct > thresh)
                {
                    const float target = thresh;

                    const float start_retract = target + 0.25f;
                    const float stop_retract  = target + 0.00f;

                    retract_hys_active = hyst_u8(retract_hys_active, pct, start_retract, stop_retract);

                    if (!retract_hys_active)
                    {
                        PID_pressure.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0.0f;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    const float err = pct - target;
                    on_use_need_move = true;
                    on_use_abs_err   = err;

                    const float mag = retract_mag_from_err(err, 850.0f);

                    x = dir * mag; // tylko cofanie
                    if (x * dir < 0.0f) x = 0.0f;
                }
                else
                {
                    retract_hys_active = 0;

                    if (pct >= 49.8f)
                    {
                        PID_pressure.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0.0f;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    // dociągnij do 50%
                    constexpr float target = 50.0f;
                    const float err = pct - target;
                    on_use_need_move = true;
                    on_use_abs_err   = -err;

                    x = dir * PID_pressure.caculate(err, time_E);

                    float lim_f = 550.0f + 90.0f * on_use_abs_err;
                    if (lim_f > 950.0f) lim_f = 950.0f;

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
            else if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use)
            {
                const float pct = MC_PULL_pct_f[CHx];

                constexpr float target_pct = 51.0f;
                constexpr float band_lo    = 50.5f;
                constexpr float band_hi    = 54.0f;

                if (post_sendout_10s)
                {
                    float thresh = post_sendout_retract_thresh_pct;
                    if (thresh < target_pct) thresh = target_pct;

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
                        }
                        else
                        {
                            const float err = pct - target;
                            on_use_need_move = true;
                            on_use_abs_err   = err;

                            const float mag = retract_mag_from_err(err, 900.0f);

                            x = dir * mag; // tylko cofanie
                            if (x * dir < 0.0f) x = 0.0f;
                        }
                    }
                    else
                    {
                        retract_hys_active = 0;

                        // push-only do target (bez cofania do targetu)
                        if (pct >= band_lo)
                        {
                            x = 0.0f;
                            PID_pressure.clear();
                            on_use_need_move = false;
                            on_use_abs_err   = 0.0f;
                        }
                        else
                        {
                            const float err = pct - target_pct;
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
                }
                else
                {
                    retract_hys_active = 0;

                    if (pct >= band_lo && pct <= band_hi)
                    {
                        x = 0.0f;
                        PID_pressure.clear();
                        on_use_need_move = false;
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
                    const float FAST = (float)MC_PULL_SEND_FAST_PCT;
                    const float STOP = (float)MC_PULL_SEND_STOP_PCT;

                    const float pct = MC_PULL_pct_f[CHx];

                    // HARD STOP
                    if (pct >= (float)MC_PULL_SEND_HARD_STOP_PCT)
                    {
                        send_hard = true;
                        send_base = 0.0f;
                        PID_speed.clear();
                        pwm_zeroed = 1;
                        x_prev[CHx] = 0.0f;
                        Motion_control_set_PWM(CHx, 0);
                        return;
                    }

                    if (send_hard)
                    {
                        if (pct >= (float)(MC_PULL_SEND_HARD_STOP_PCT - MC_PULL_SEND_HARD_HYS))
                        {
                            PID_speed.clear();
                            pwm_zeroed = 1;
                            x_prev[CHx] = 0.0f;
                            Motion_control_set_PWM(CHx, 0);
                            return;
                        }
                        send_hard = false;
                    }

                    const float dpct = pct - send_pct_prev;
                    send_pct_prev = pct;

                    send_hold = (pct >= FAST);

                    // latch przy STOP
                    if (!send_stop_latch && pct >= (float)MC_PULL_SEND_STOP_PCT) {
                        send_stop_latch = true;
                        send_stop_brake_ms = now_ms;
                        send_base = 0.0f;
                        PID_speed.clear();
                        PID_pressure.clear();
                    }

                    if (send_stop_latch)
                    {
                        do_speed_pid = false;

                        constexpr float target  = 53.0f;
                        constexpr float wait_hi = 53.2f;
                        constexpr float band_lo = 52.6f;
                        constexpr float band_hi = 53.4f;

                        if (pct > wait_hi)
                        {
                            x = 0.0f;
                            PID_pressure.clear();
                            PID_speed.clear();
                            on_use_need_move = false;
                            on_use_abs_err   = 0.0f;
                        }
                        else
                        {
                            if (pct >= band_lo && pct <= band_hi)
                            {
                                x = 0.0f;
                                PID_pressure.clear();
                                on_use_need_move = false;
                                on_use_abs_err   = 0.0f;
                            }
                            else
                            {
                                on_use_need_move = true;

                                const float err = pct - target;
                                on_use_abs_err = (err < 0.0f) ? -err : err;

                                x = dir * PID_pressure.caculate(err, time_E);

                                float lim_f = 520.0f + 110.0f * on_use_abs_err;
                                if (lim_f > 950.0f) lim_f = 950.0f;

                                if (x >  lim_f) x =  lim_f;
                                if (x < -lim_f) x = -lim_f;
                            }
                        }

                    }

                    if (!send_stop_latch)
                    {
                        if (!send_hold)
                        {
                            speed_set = 50.0f;
                        }
                        else
                        {
                            const float e = STOP - pct;

                            float u = (STOP - pct) / (STOP - FAST);
                            if (u < 0.0f) u = 0.0f;
                            if (u > 1.0f) u = 1.0f;
                            const float sp_ff = 25.0f * u;

                            constexpr float Ki = 70.0f;
                            constexpr float base_max = 40.0f;

                            if (dpct > 0.0f)
                            {
                                float k = time_E / 0.07f;
                                if (k > 1.0f) k = 1.0f;
                                send_base -= send_base * k;
                            }
                            else
                            {
                                if (e > 0.0f && pct < ((float)MC_PULL_SEND_STOP_PCT - 0.8f))
                                    send_base += Ki * e * time_E;
                            }

                            if (send_base < 0.0f) send_base = 0.0f;
                            if (send_base > base_max) send_base = base_max;

                            constexpr float Kp = 4.0f;
                            float sp_pi = Kp * e + send_base;

                            float sp = (e <= 0.0f) ? 0.0f : sp_pi;
                            if (sp < sp_ff) sp = sp_ff;

                            if (e <= 5.0f)
                            {
                                if (sp > 9.0f) sp = 9.0f;
                                if (sp < 0.0f) sp = 0.0f;
                            }

                            if (sp > 25.0f) sp = 25.0f;
                            if (sp < 0.0f)  sp = 0.0f;

                            speed_set = sp;
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
        const bool pb_mode = (motion == filament_motion_enum::filament_motion_before_pull_back);

        const bool hold_mode =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) ||
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) ||
            (motion == filament_motion_enum::filament_motion_before_on_use) ||
            (motion == filament_motion_enum::filament_motion_stop_on_use) ||
            post_on_use_active;

        const bool send_hold_mode = (motion == filament_motion_enum::filament_motion_send) && send_hold;

        const bool pull_mode = (motion == filament_motion_enum::filament_motion_pull);

        const int deadband =
            pb_mode ? 0 :
            (hold_mode ? 1 : (send_hold_mode ? 6 : (pull_mode ? 2 : 10)));

        float pwm0 =
            pb_mode ? 0.0f :
            (hold_mode ? 300.0f :
             (send_hold_mode ? 350.0f : pwm_zero));

        if (pull_mode)
        {
            float k = g_pull_remain_m[CHx] / PULL_RAMP_M;
            k = clampf(k, 0.0f, 1.0f);

            // daleko: ~pwm_zero (500), przy końcu: >=400
            pwm0 = PULL_PWM_MIN + (pwm_zero - PULL_PWM_MIN) * k;

            if (pwm0 < PULL_PWM_MIN) pwm0 = PULL_PWM_MIN;
        }

        // friction offset
        if (x > (float)deadband)       x += pwm0;
        else if (x < (float)-deadband) x -= pwm0;
        else                           x = 0.0f;

        // clamp
        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)
        {
            constexpr float PWM_IDLE_LIM = 800.0f;
            if (x >  PWM_IDLE_LIM) x =  PWM_IDLE_LIM;
            if (x < -PWM_IDLE_LIM) x = -PWM_IDLE_LIM;
        } else {
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
                const int MIN_MOVE_PWM = (on_use_abs_err >= 1.3f) ? 500 : 0;
                if (MIN_MOVE_PWM)
                {
                    int xi = (int)(x + ((x >= 0.0f) ? 0.5f : -0.5f));
                    const int ax = (xi < 0) ? -xi : xi;
                    if (ax < MIN_MOVE_PWM)
                        x = (x > 0.0f) ? (float)MIN_MOVE_PWM : (float)-MIN_MOVE_PWM;
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

        if (motion == filament_motion_enum::filament_motion_redetect)
        {
            const int pwm_out = (int)x;
            pwm_zeroed = (pwm_out == 0);
            x_prev[CHx] = x;
            Motion_control_set_PWM(CHx, pwm_out);
            return;
        }

        const bool use_ramping =
            (motion == filament_motion_enum::filament_motion_send) ||
            (motion == filament_motion_enum::filament_motion_pull);

        if (use_ramping)
        {
            const bool send_soft_start =
                (motion == filament_motion_enum::filament_motion_send) &&
                (send_start_ms != 0) &&
                ((now_ms - send_start_ms) < 400ull);

            const bool pull_soft_start =
                (motion == filament_motion_enum::filament_motion_pull) &&
                (pull_start_ms != 0) &&
                ((now_ms - pull_start_ms) < 400ull);

            float rate_up   = 4500.0f;
            float rate_down = 6500.0f;

            if (send_soft_start && !send_hold) rate_up = 2500.0f;
            if (pull_soft_start)               rate_up = 2500.0f;

            if (motion == filament_motion_enum::filament_motion_send)
            {
                rate_down = 25000.0f;

                if (send_hold)
                {
                    rate_up = 3000.0f;
                }
            }

            float max_step_up   = rate_up   * time_E;
            float max_step_down = rate_down * time_E;

            if (max_step_up   < 1.0f) max_step_up   = 1.0f;
            if (max_step_down < 1.0f) max_step_down = 1.0f;


            const float prev = x_prev[CHx];
            float lo = prev - max_step_down;
            float hi = prev + max_step_up;

            if (x < lo) x = lo;
            if (x > hi) x = hi;
        }


        const int pwm_out = (int)x;
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

void AS5600_distance_updata()
{
    static uint32_t last_us = 0;
    static uint8_t  was_ok[4] = {0,0,0,0};

    static uint32_t last_stu_ms = 0;

    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_stu_ms) >= 200u)
    {
        last_stu_ms = now_ms;
        MC_AS5600.updata_stu();
    }

    const uint32_t now_us = micros();
    if (last_us == 0) { last_us = now_us; return; }

    const uint32_t dt_us = (uint32_t)(now_us - last_us);
    if (dt_us == 0) return;
    last_us = now_us;

    const float inv_dt = 1000000.0f / (float)dt_us;

    MC_AS5600.updata_angle();
    auto &A = ams[motion_control_ams_num];

    for (uint8_t i = 0; i < kChCount; i++)
    {
        const bool ok_now = MC_AS5600.online[i] && (MC_AS5600.magnet_stu[i] != AS5600_soft_IIC_many::offline);

        if (ok_now)
        {
            g_as5600_fail[i] = 0;
            if (g_as5600_okstreak[i] < 255) g_as5600_okstreak[i]++;

            if (g_as5600_okstreak[i] >= kAS5600_OK_RECOVER)
                g_as5600_good[i] = 1;
        }
        else
        {
            g_as5600_okstreak[i] = 0;
            if (g_as5600_fail[i] < 255) g_as5600_fail[i]++;

            if (g_as5600_fail[i] >= kAS5600_FAIL_TRIP)
                g_as5600_good[i] = 0;
        }

        if (!AS5600_is_good(i))
        {
            was_ok[i] = 0;
            speed_as5600[i] = 0.0f;
            continue;
        }

        if (!was_ok[i])
        {
            as5600_distance_save[i] = MC_AS5600.raw_angle[i];
            speed_as5600[i] = 0.0f;
            was_ok[i] = 1;
            continue;
        }

        const int32_t last = as5600_distance_save[i];
        const int32_t now  = MC_AS5600.raw_angle[i];

        int32_t diff = now - last;
        if (diff >  2048) diff -= 4096;
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
            MC_STU_RGB_set(i, 0xFF, 0x00, 0xFF);

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
            MC_STU_RGB_set(i, 0xFF, 0xFF, 0x00);

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
                MC_STU_RGB_set(num, 0xFF, 0xFF, 0x00);
                break;
            }

            case _filament_motion::stop_on_use:
            {
                filament_now_position[num] = filament_using;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_stop_on_use, 300, time_now);
                MC_STU_RGB_set(num, 0xFF, 0x00, 0x00);
                break;
            }
            case _filament_motion::send_out:
                MC_STU_RGB_set(num, 0x00, 0xD5, 0x2A);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100, time_now);
                break;

            case _filament_motion::pull_back:
            {
                MC_STU_RGB_set(num, 0xA0, 0x2D, 0xFF);
                filament_now_position[num] = filament_pulling_back;

                filament_pull_back_meters[num] = A.filament[num].meters;

                float already = before_pb_retracted_m[num];
                float target  = motion_control_pull_back_distance - already;

                if (target < 0.0f) target = 0.0f;
                if (target > motion_control_pull_back_distance) target = motion_control_pull_back_distance;

                filament_pull_back_target[num] = target;

                g_pull_remain_m[num]  = target;
                g_pull_speed_set[num] = -PULL_V_FAST;

                // pullback czyści pamięć before_pull_back
                before_pb_retracted_m[num] = 0.0f;
                before_pb_sign[num]        = 0;
                before_pb_last_m[num]      = filament_pull_back_meters[num];

                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100, time_now);
                break;
            }

            case _filament_motion::before_pull_back:
            {
                MC_STU_RGB_set(num, 0xFF, 0xA0, 0x00);

                if (filament_now_position[num] != filament_before_pull_back)
                {
                    filament_now_position[num] = filament_before_pull_back;
                    before_pb_last_m[num]      = A.filament[num].meters;
                    before_pb_retracted_m[num] = 0.0f;
                    before_pb_sign[num]        = 0;
                }

                // cofanie
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
                MC_STU_RGB_set(num, 0x00, 0xB0, 0xFF);
                break;
            }

            case _filament_motion::idle:
            default:
                filament_now_position[num] = filament_idle;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100, time_now);
                MC_STU_RGB_set(num, 0x38, 0x35, 0x32);
                break;
            }
        }
        else
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100, time_now);
            MC_STU_RGB_set(num, 0x38, 0x35, 0x32);
        }
    }
}

static void motor_motion_run(int error)
{
    const uint64_t time_now = get_time64();
    static uint64_t time_last = 0;

    if (time_last == 0) { time_last = time_now; return; }

    uint64_t dt_ms = time_now - time_last;
    if (dt_ms == 0) return;
    if (dt_ms > 200) dt_ms = 200;

    const float time_E = (float)dt_ms * 0.001f;
    time_last = time_now;

    const uint16_t device_type = bus_host_device_type;

    if (!error)
    {
        if (device_type == host_device_type_ams_lite)
        {
            motor_motion_switch(time_now);
        }
        else if (device_type == host_device_type_ams)
        {
            if (!motor_motion_filamnet_pull_back_to_online_key(time_now))
                motor_motion_switch(time_now);
        }
        else if (device_type == host_device_type_ahub)
        {
            motor_motion_switch(time_now);
        }
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
        MOTOR_CONTROL[i].run(time_E, time_now);

        if (MC_PULL_stu[i] == 1) {
            MC_PULL_ONLINE_RGB_set(i, 0x10, 0x00, 0x00);
        } else if (MC_PULL_stu[i] == -1) {
            MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0x10);
        } else {
            // ===== VISUAL LED FEEDBACK FOR DUAL MICROSWITCH =====
            if (IS_TWO_MICROSWITCH_ENABLED)
            {
                // Dual microswitch system - distinctive LED for each state
                if (MC_ONLINE_key_stu[i] == 2)
                {
                    // Only EXTERNAL microswitch - Assistance active (Orange/Gold)
                    MC_PULL_ONLINE_RGB_set(i, 0xFF, 0x90, 0x00);
                }
                else if (MC_ONLINE_key_stu[i] == 1)
                {
                    // BOTH microswitches - Filament fully inserted (Blue)
                    MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0xFF);
                }
                else if (MC_ONLINE_key_stu[i] == 3)
                {
                    // Only INTERNAL microswitch - Anomaly situation (Cyan)
                    MC_PULL_ONLINE_RGB_set(i, 0x00, 0xFF, 0xFF);
                }
                else // MC_ONLINE_key_stu[i] == 0
                {
                    // No filament - Offline (Off or dim blue)
                    MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0x10);
                }
            }
            else
            {
                // Single microswitch system - original behavior
                if (MC_ONLINE_key_stu[i] != 0) {
                    MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0x00);
                } else {
                    const uint8_t pct = MC_PULL_pct[i];
                    if ((uint8_t)(pct - 49u) <= 2u) MC_PULL_ONLINE_RGB_set(i, 0x10, 0x08, 0x00);
                    else MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0x00);
                }
            }
        }
    }
}

void Motion_control_run(int error)
{
    MC_PULL_ONLINE_read();

    // long-press reset kalibracji (tylko gdy brak filamentu na wszystkich kanałach)
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

        const uint32_t tpm = time_hw_ticks_per_ms();
        const uint32_t now_t = time_ticks32();

        if (pressed >= 0)
        {
            if (g_hold_ch != pressed) {
                g_hold_ch = pressed;
                g_hold_t0_ticks = now_t;
            } else {
                if ((uint32_t)(now_t - g_hold_t0_ticks) >= (uint32_t)CAL_RESET_HOLD_MS * tpm)
                    calibration_reset_and_reboot();
            }
        }
        else
        {
            g_hold_ch = -1;
            g_hold_t0_ticks = 0;
        }
    }
    else
    {
        g_hold_ch = -1;
        g_hold_t0_ticks = 0;
    }

    AS5600_distance_updata();

    auto &A = ams[motion_control_ams_num];

    for (uint8_t i = 0; i < kChCount; i++)
    {
        if (MC_ONLINE_key_stu[i] != 0) A.filament[i].online = true;
        else if ((filament_now_position[i] == filament_redetect) || (filament_now_position[i] == filament_pulling_back))
            A.filament[i].online = true;
        else
            A.filament[i].online = false;
    }

    if (error)
    {
        for (uint8_t i = 0; i < kChCount; i++)
        {
            A.filament[i].online = false;
            if (MC_ONLINE_key_stu[i] != 0) MC_STU_RGB_set(i, 0x38, 0x35, 0x32);
            else MC_STU_RGB_set(i, 0x00, 0x00, 0x00);
        }
    }
    else
    {
        for (uint8_t i = 0; i < kChCount; i++)
        {
            if (MC_ONLINE_key_stu[i] != 0 && filament_channel_inserted[i])
                MC_STU_RGB_set(i, 0x38, 0x35, 0x32);
            else
                MC_STU_RGB_set(i, 0x00, 0x00, 0x00);
        }
    }

    motor_motion_run(error);

    for (uint8_t i = 0; i < kChCount; i++)
    {
        // AS5600 error
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // zegar AFIO
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // baza timera
    TIM_TimeBaseStructure.TIM_Period        = 999;
    TIM_TimeBaseStructure.TIM_Prescaler     = 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM1
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3

    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5

    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2 full remap: CH1-PA15 / CH2-PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3 partial: CH1-PB4 / CH2-PB5
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4 no remap

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

    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read();

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
