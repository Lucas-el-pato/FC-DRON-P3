/**
 ******************************************************************************
 * @file    fc_rc.c
 * @brief   Implementacion del mapeo de canales CRSF (ver fc_rc.h).
 ******************************************************************************
 */

#include "fc_rc.h"
#include "stm32f4xx_hal.h"

/* Frames que se consumen como maximo por pasada (evita quedarse en el ring
 * si el receptor manda mas rapido de lo que el lazo consume). */
#define FC_RC_MAX_FRAMES_PER_POLL  4u

fc_rc_t g_fcRc;

static uint32_t s_last_rc_ms = 0u;
static bool     s_have_rc = false;

/* Canal crudo -> -1..+1 con deadband y expo. */
static float fc_rc_axis_norm(uint16_t raw)
{
    int32_t delta = (int32_t)raw - (int32_t)FC_RC_RAW_MID;

    if (delta > (int32_t)FC_RC_DEADBAND) {
        delta -= (int32_t)FC_RC_DEADBAND;
    } else if (delta < -(int32_t)FC_RC_DEADBAND) {
        delta += (int32_t)FC_RC_DEADBAND;
    } else {
        return 0.0f;
    }

    const float span = (float)(FC_RC_RAW_MAX - FC_RC_RAW_MID - FC_RC_DEADBAND);
    float norm = (float)delta / span;

    if (norm > 1.0f) {
        norm = 1.0f;
    } else if (norm < -1.0f) {
        norm = -1.0f;
    }

    /* Expo cubico: mas resolucion cerca del centro. */
    return (1.0f - FC_RC_EXPO) * norm + FC_RC_EXPO * norm * norm * norm;
}

static float fc_rc_throttle_norm(uint16_t raw)
{
    if (raw <= FC_RC_RAW_MIN) {
        return 0.0f;
    }
    if (raw >= FC_RC_RAW_MAX) {
        return 1.0f;
    }
    return (float)(raw - FC_RC_RAW_MIN) / (float)(FC_RC_RAW_MAX - FC_RC_RAW_MIN);
}

void fc_rc_init(void)
{
    for (uint8_t i = 0u; i < CRSF_CHANNELS_COUNT; ++i) {
        g_fcRc.raw[i] = (uint16_t)FC_RC_RAW_MID;
    }
    g_fcRc.raw[FC_RC_CH_THROTTLE] = (uint16_t)FC_RC_RAW_MIN;
    g_fcRc.raw[FC_RC_CH_ARM] = (uint16_t)FC_RC_RAW_MIN;

    g_fcRc.throttle = 0.0f;
    for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
        g_fcRc.setpoint_dps[ax] = 0.0f;
    }
    g_fcRc.arm_switch = false;
    g_fcRc.link_ok = false;
    g_fcRc.age_ms = FC_RC_LINK_TIMEOUT_MS;
    g_fcRc.frames = 0u;

    s_last_rc_ms = HAL_GetTick();
    s_have_rc = false;

    crsf_init();
}

static void fc_rc_apply_channels(const crsf_channels_t *ch)
{
    for (uint8_t i = 0u; i < CRSF_CHANNELS_COUNT; ++i) {
        g_fcRc.raw[i] = ch->ch[i];
    }

    g_fcRc.throttle = fc_rc_throttle_norm(ch->ch[FC_RC_CH_THROTTLE]);

    g_fcRc.setpoint_dps[PID_AXIS_ROLL] =
        fc_rc_axis_norm(ch->ch[FC_RC_CH_ROLL]) * FC_RC_RATE_DPS;
    g_fcRc.setpoint_dps[PID_AXIS_PITCH] =
        fc_rc_axis_norm(ch->ch[FC_RC_CH_PITCH]) * FC_RC_RATE_DPS;
    g_fcRc.setpoint_dps[PID_AXIS_YAW] =
        fc_rc_axis_norm(ch->ch[FC_RC_CH_YAW]) * FC_RC_YAW_RATE_DPS;

    g_fcRc.arm_switch = (ch->ch[FC_RC_CH_ARM] > (uint16_t)FC_RC_ARM_THRESHOLD);
    g_fcRc.frames++;
}

bool fc_rc_poll(void)
{
    bool got_channels = false;
    crsf_frame_t frame;

    for (uint8_t i = 0u; i < FC_RC_MAX_FRAMES_PER_POLL; ++i) {
        if (crsf_poll_frame(&frame) != CRSF_OK) {
            break;   /* sin frame completo o CRC malo: se reintenta luego */
        }
        if (frame.type != CRSF_TYPE_RC_CHANNELS) {
            continue;
        }

        crsf_channels_t ch;
        if (crsf_decode_channels(&frame, &ch) != CRSF_OK) {
            continue;
        }

        fc_rc_apply_channels(&ch);
        s_last_rc_ms = HAL_GetTick();
        s_have_rc = true;
        got_channels = true;
    }

    fc_rc_update_link();
    return got_channels;
}

void fc_rc_update_link(void)
{
    const uint32_t age = HAL_GetTick() - s_last_rc_ms;

    g_fcRc.age_ms = age;
    g_fcRc.link_ok = s_have_rc && (age < FC_RC_LINK_TIMEOUT_MS);

    if (!g_fcRc.link_ok) {
        /* Sin enlace no se conservan setpoints viejos. */
        g_fcRc.throttle = 0.0f;
        for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
            g_fcRc.setpoint_dps[ax] = 0.0f;
        }
        g_fcRc.arm_switch = false;
    }
}

const fc_rc_t *fc_rc_get(void)
{
    return &g_fcRc;
}
