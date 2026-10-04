#include "MC_PULL_calibration.h"
#include "Motion_control.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "hal/time_hw.h"
#include "Debug_log.h"
#include "app_api.h"
#include <math.h>

static uint8_t g_valid_mask = 0u;

bool MC_PULL_calibration_is_valid(uint8_t ch)
{
    return ch < 4u && (g_valid_mask & (1u << ch));
}

static bool finite_voltage(float v)
{
    return isfinite(v) && v >= 0.05f && v <= 3.25f;
}

extern void RGB_update();

static inline float adc_pull_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[6];
    case 1: return (float)v8[4];
    case 2: return (float)v8[2];
    default:return (float)v8[0];
    }
}

static inline float adc_key_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[7];
    case 1: return (float)v8[5];
    case 2: return (float)v8[3];
    default:return (float)v8[1];
    }
}

static inline uint8_t dm_key_round_up_to_centi(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

static inline float dm_key_none_threshold_from_idle(float key_value)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_value);

    uint8_t thr_cv = (uint8_t)(key_cv + 10u);
    if (thr_cv < 60u) thr_cv = 60u;
    if (thr_cv > 139u) thr_cv = 139u;

    return 0.01f * (float)thr_cv;
}

static inline float cal_apply_polarity(float v, int8_t pol)
{
    return (pol < 0) ? (3.30f - v) : v;
}

static const float    CAL_PRESS_DELTA_V = 0.1f;
static const float    CAL_CENTER_EPS_V  = 0.025f;
static const uint32_t CAL_STABLE_MS     = 700;
static const uint32_t CAL_TIMEOUT_MS    = 30000;

static void blink_all(uint8_t r, uint8_t g, uint8_t b, int times = 4, int on_ms = 60, int off_ms = 60)
{
    for (int k = 0; k < times; k++)
    {
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

static void blink_one(int ch, uint8_t r, uint8_t g, uint8_t b, int times = 3, int on_ms = 70, int off_ms = 70)
{
    for (int k = 0; k < times; k++)
    {
        MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

static bool capture_first_extreme_wait_release(int ch, float center_v, float offset, float &out_best_norm, int8_t &out_pol)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;

    bool pressed = false;
    float raw_min = center_v;
    float raw_max = center_v;
    uint32_t stable_t0 = 0u;

    out_best_norm = center_v;
    out_pol = 1;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;

        const float raw = adc_pull_raw_ch(ch, ADC_DMA_get_value());
        if (!finite_voltage(raw)) break;
        const float v = raw + offset;

        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x10);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            if (fabsf(v - center_v) >= CAL_PRESS_DELTA_V)
            {
                pressed = true;
                raw_min = v;
                raw_max = v;
            }
        }
        else
        {
            if (v < raw_min) raw_min = v;
            if (v > raw_max) raw_max = v;

            if (fabsf(v - center_v) <= CAL_CENTER_EPS_V)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    const float dmin = center_v - raw_min;
                    const float dmax = raw_max - center_v;

                    if (dmax > dmin)
                    {
                        out_pol = -1;
                        out_best_norm = cal_apply_polarity(raw_max, out_pol);
                    }
                    else
                    {
                        out_pol = 1;
                        out_best_norm = cal_apply_polarity(raw_min, out_pol);
                    }

                    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
                    RGB_update();
                    return true;
                }
            }
            else
            {
                stable_t0 = 0u;
            }
        }

        delay(15);
    }

    out_best_norm = center_v;
    out_pol = 1;
    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    return false;
}

static bool capture_second_extreme_wait_release(int ch, float center_v, float offset, int8_t pol, float &out_best_norm)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;

    const bool want_raw_min = (pol < 0);

    bool pressed = false;
    float best_raw = center_v;
    uint32_t stable_t0 = 0u;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;

        const float raw = adc_pull_raw_ch(ch, ADC_DMA_get_value());
        if (!finite_voltage(raw)) break;
        const float v = raw + offset;

        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x10, 0x00, 0x00);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            if (want_raw_min)
            {
                if (v < (center_v - CAL_PRESS_DELTA_V))
                {
                    pressed = true;
                    best_raw = v;
                }
            }
            else
            {
                if (v > (center_v + CAL_PRESS_DELTA_V))
                {
                    pressed = true;
                    best_raw = v;
                }
            }
        }
        else
        {
            if (want_raw_min)
            {
                if (v < best_raw) best_raw = v;
            }
            else
            {
                if (v > best_raw) best_raw = v;
            }

            if (fabsf(v - center_v) <= CAL_CENTER_EPS_V)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    out_best_norm = cal_apply_polarity(best_raw, pol);
                    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
                    RGB_update();
                    return true;
                }
            }
            else
            {
                stable_t0 = 0u;
            }
        }

        delay(15);
    }

    out_best_norm = center_v;
    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    return false;
}

static bool capture_minmax_one_ch_event(int ch, float center_v, float offset, float &out_min, float &out_max, int8_t &out_pol)
{
    float vmin = center_v;
    float vmax = center_v;
    int8_t pol = 1;

    bool ok_min = capture_first_extreme_wait_release(ch, center_v, offset, vmin, pol);
    if (!ok_min) return false;
    blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);

    bool ok_max = capture_second_extreme_wait_release(ch, center_v, offset, pol, vmax);
    if (!ok_max) return false;
    blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);

    if (!isfinite(vmin) || !isfinite(vmax) ||
        center_v - vmin < 0.050f || vmax - center_v < 0.050f ||
        vmax - vmin < 0.120f) return false;

    out_min = vmin;
    out_max = vmax;
    out_pol = pol;
    return true;
}

bool MC_PULL_calibration_clear()
{
    g_valid_mask = 0u;
    return Flash_MC_PULL_cal_clear();
}

void MC_PULL_calibration_boot()
{
    g_valid_mask = 0u;
    for (int i = 0; i < 6; i++) { ADC_DMA_poll(); delay(20); }
    MC_PULL_detect_channels_inserted();
    uint8_t selected = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++)
        if (filament_channel_inserted[ch]) selected |= (uint8_t)(1u << ch);
    if (!selected) return;

    float offs[4] = {}, vmin[4] = {}, vmax[4] = {}, dm_none[4] = {};
    int8_t pol[4] = {1, 1, 1, 1};
    uint8_t valid = 0u;
    bool stored = Flash_MC_PULL_cal_read(offs, vmin, vmax, pol, dm_none, &valid);
    if (stored)
    {
        for (uint8_t ch = 0u; ch < 4u; ch++)
        {
            if (!(valid & (1u << ch))) continue;
            if (!isfinite(offs[ch]) || !finite_voltage(1.65f - offs[ch]) ||
                !isfinite(vmin[ch]) || !isfinite(vmax[ch]) ||
                1.65f - vmin[ch] < 0.050f || vmax[ch] - 1.65f < 0.050f ||
                vmax[ch] - vmin[ch] < 0.120f ||
                !isfinite(dm_none[ch]) || dm_none[ch] < 0.01f * 60.0f || dm_none[ch] > 0.01f * 139.0f)
                valid &= (uint8_t)~(1u << ch);
        }
        stored = (valid & selected) == selected;
    }

    if (!stored)
    {
        float sum_raw[4] = {}, sum_key[4] = {};
        float low[4] = {10, 10, 10, 10}, high[4] = {};
        uint8_t baseline_valid = selected;
        const int samples = 60;
        for (int k = 0; k < samples; k++)
        {
            const float *v = ADC_DMA_get_value();
            for (uint8_t ch = 0u; ch < 4u; ch++)
            {
                if (!(selected & (1u << ch))) continue;
                const float raw = adc_pull_raw_ch(ch, v);
                const float key = adc_key_raw_ch(ch, v);
                if (!finite_voltage(raw) || !isfinite(key) || key < 0.0f || key >= 1.40f)
                    baseline_valid &= (uint8_t)~(1u << ch);
                sum_raw[ch] += raw;
                sum_key[ch] += key;
                if (raw < low[ch]) low[ch] = raw;
                if (raw > high[ch]) high[ch] = raw;
                const bool on = ((k / 15) & 1) == 0;
                MC_PULL_ONLINE_RGB_set(ch, on ? 0x0Cu : 0u, on ? 0x0Cu : 0u, 0u);
            }
            RGB_update();
            delay(15);
        }
        for (uint8_t ch = 0u; ch < 4u; ch++)
        {
            offs[ch] = 0.0f;
            vmin[ch] = 1.50f;
            vmax[ch] = 1.80f;
            dm_none[ch] = 0.60f;
            pol[ch] = 1;
            if (!(selected & (1u << ch))) continue;
            if (high[ch] - low[ch] > 0.040f) baseline_valid &= (uint8_t)~(1u << ch);
            if (!(baseline_valid & (1u << ch))) continue;
            offs[ch] = 1.65f - sum_raw[ch] / (float)samples;
            dm_none[ch] = dm_key_none_threshold_from_idle(sum_key[ch] / (float)samples);
            MC_PULL_ONLINE_RGB_set(ch, 0u, 0u, 0u);
        }
        RGB_update();
        if (baseline_valid != selected) { blink_all(0x18u, 0u, 0u, 2, 220, 220); return; }

        valid = 0u;
        for (uint8_t ch = 0u; ch < 4u; ch++)
        {
            if (!(selected & (1u << ch))) continue;
            if (!capture_minmax_one_ch_event(ch, 1.65f, offs[ch], vmin[ch], vmax[ch], pol[ch]))
            {
                blink_all(0x18u, 0u, 0u, 2, 220, 220);
                return;
            }
            valid |= (uint8_t)(1u << ch);
            MC_PULL_ONLINE_RGB_set(ch, 0x10u, 0x10u, 0u);
            RGB_update();
        }

        const uint32_t started = time_ticks32();
        uint32_t centered_since = 0u;
        bool released = false;
        while ((uint32_t)(time_ticks32() - started) < ms_to_ticks32(CAL_TIMEOUT_MS))
        {
            const float *v = ADC_DMA_get_value();
            bool centered = true;
            for (uint8_t ch = 0u; ch < 4u; ch++)
            {
                if (!(selected & (1u << ch))) continue;
                const float raw = adc_pull_raw_ch(ch, v);
                const float key = adc_key_raw_ch(ch, v);
                if (!finite_voltage(raw) || !isfinite(key) || key < 0.0f || key >= dm_none[ch] ||
                    fabsf(raw + offs[ch] - 1.65f) > 0.080f) centered = false;
            }
            const uint32_t now = time_ticks32();
            if (!centered) centered_since = 0u;
            else if (!centered_since) centered_since = now;
            else if ((uint32_t)(now - centered_since) >= ms_to_ticks32(1200u))
            {
                released = true;
                break;
            }
            delay(15);
        }
        if (!released || valid != selected ||
            !Flash_MC_PULL_cal_write_all(offs, vmin, vmax, pol, dm_none, valid))
        {
            blink_all(0x18u, 0u, 0u, 2, 220, 220);
            return;
        }
    }

    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (!(valid & (1u << ch))) continue;
        MC_PULL_V_OFFSET[ch] = offs[ch];
        MC_PULL_V_MIN[ch] = vmin[ch];
        MC_PULL_V_MAX[ch] = vmax[ch];
        MC_PULL_POLARITY[ch] = pol[ch];
        MC_DM_KEY_NONE_THRESH[ch] = dm_none[ch];
    }
    g_valid_mask = valid & selected;
    if (!stored) blink_all(0u, 0x16u, 0u, 2, 220, 220);
}
