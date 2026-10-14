/**
 ******************************************************************************
 * @file    sensors_battery.c
 * @brief   Voltaje de bateria por ADC (ver sensors_battery.h).
 ******************************************************************************
 */

#include "sensors_battery.h"
#include "adc.h"

#include <math.h>

/* Pasabajos ~1 Hz con la tarea a 10 Hz: k = dt / (dt + 1/(2*pi*fc)). */
#define VBAT_LPF_K   0.386f

static sensors_battery_t s_bat;

void sensors_battery_init(void)
{
    s_bat.voltage_v = 0.0f;
    s_bat.voltage_cv = 0u;
    s_bat.raw = 0u;
    s_bat.cells = 0u;
    s_bat.percent = 0u;
    s_bat.valid = false;
}

void sensors_battery_update(void)
{
    const uint16_t raw = adc_vbat_read_raw();
    if (raw == 0xFFFFu) {
        return;
    }
    s_bat.raw = raw;

    const float v = (float)raw * ((float)ADC_VREF_MV * 0.001f / (float)ADC_MAX_COUNTS) *
                    SENSORS_VBAT_DIVIDER * SENSORS_VBAT_TRIM;

    if (!s_bat.valid) {
        s_bat.voltage_v = v;
        s_bat.valid = true;
    } else {
        s_bat.voltage_v += VBAT_LPF_K * (v - s_bat.voltage_v);
    }
    s_bat.voltage_cv = (uint16_t)lroundf(s_bat.voltage_v * 100.0f);

    /* Celdas: se detectan una vez con bateria presente (como BF al conectar). */
    if (s_bat.cells == 0u && s_bat.voltage_v > SENSORS_VBAT_PRESENT_V) {
        s_bat.cells = (uint8_t)(s_bat.voltage_v / SENSORS_VBAT_CELL_MAX_V) + 1u;
    } else if (s_bat.voltage_v < SENSORS_VBAT_PRESENT_V) {
        s_bat.cells = 0u;   /* bateria desconectada: re-detectar al volver */
    }

    if (s_bat.cells == 0u) {
        s_bat.percent = 0u;
        return;
    }

    const float cell_v = s_bat.voltage_v / (float)s_bat.cells;
    float pct = (cell_v - SENSORS_VBAT_CELL_EMPTY_V) * 100.0f /
                (SENSORS_VBAT_CELL_FULL_V - SENSORS_VBAT_CELL_EMPTY_V);
    if (pct < 0.0f) {
        pct = 0.0f;
    } else if (pct > 100.0f) {
        pct = 100.0f;
    }
    s_bat.percent = (uint8_t)pct;
}

const sensors_battery_t *sensors_battery_get(void)
{
    return &s_bat;
}
