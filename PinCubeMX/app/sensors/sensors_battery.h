/**
 ******************************************************************************
 * @file    sensors_battery.h
 * @brief   Voltaje de bateria por ADC (equivalente a sensors/battery.c de
 *          Betaflight, solo voltaje).
 *
 *          Hardware (hardware/esquematico/power.kicad_sch):
 *            Vbat -> R17 100k -> Bat_Med_FC (PB0 / ADC1_IN8) -> R18 15k -> GND
 *          Factor del divisor (100k + 15k) / 15k = 7.667: 6S lleno (25.2 V)
 *          deja 3.29 V en el pin, justo bajo Vref = 3.3 V.
 *
 *          SENSORS_VBAT_TRIM corrige tolerancia de resistencias y de Vref:
 *          TRIM = V_multimetro / V_leido (con TRIM = 1).
 *
 *          Celdas: como BF, cells = V / 4.30 + 1 con la primera lectura
 *          valida (23.8 V -> 6S). % restante lineal entre 3.30 y 4.20 V/celda.
 ******************************************************************************
 */

#ifndef SENSORS_SENSORS_BATTERY_H_
#define SENSORS_SENSORS_BATTERY_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORS_VBAT_DIVIDER      ((100.0f + 15.0f) / 15.0f)

#ifndef SENSORS_VBAT_TRIM
#define SENSORS_VBAT_TRIM         1.0f
#endif

/* Por debajo de esto no hay bateria (solo USB): no se detectan celdas. */
#define SENSORS_VBAT_PRESENT_V    2.0f

#define SENSORS_VBAT_CELL_MAX_V   4.30f   /* vbatmaxcellvoltage (deteccion) */
#define SENSORS_VBAT_CELL_FULL_V  4.20f
#define SENSORS_VBAT_CELL_EMPTY_V 3.30f

typedef struct {
    float    voltage_v;    /* filtrado */
    uint16_t voltage_cv;   /* centivolts (2383 = 23.83 V) */
    uint16_t raw;          /* ultima lectura ADC, 0..4095 */
    uint8_t  cells;        /* 0 = sin bateria detectada */
    uint8_t  percent;      /* 0..100 */
    bool     valid;
} sensors_battery_t;

void sensors_battery_init(void);

/* Lee el ADC y actualiza el filtro. Llamar a ~10 Hz. */
void sensors_battery_update(void);

const sensors_battery_t *sensors_battery_get(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_SENSORS_BATTERY_H_ */
