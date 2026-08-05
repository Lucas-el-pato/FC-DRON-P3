/**
 ******************************************************************************
 * @file    test_motors.c
 * @brief   Bench test de un ESC Hobbywing con DShot300/600 (TIM2 + DMA).
 *
 *          ATENCION DE SEGURIDAD:
 *            - SACAR LAS HELICES antes de energizar.
 *            - Conectar solo la senal del ESC seleccionado (+ GND compartido).
 *            - Flashear y arrancar el firmware ANTES de alimentar el ESC:
 *              Hobbywing detecta el protocolo al encender.
 *            - Si cambia SelectEsc o el protocolo, cortar alimentacion del ESC,
 *              reflash, arrancar MCU, y recien entonces alimentar el ESC.
 *
 *          Configuracion compile-time (abajo):
 *            - SelectEsc: 1=M1 .. 4=M4 (default 1 = M1/PA15)
 *            - MOTORS_TEST_PROTOCOL: DShot300 (default) o DShot600
 *
 *          Secuencia (alineada al test ESP32 BrushlessTest):
 *            1. Aviso props-off + power-cycle note
 *            2. motors_init(protocol, SelectEsc)
 *            3. 5 s de throttle=0 (armado)
 *            4. Ramp low controlado solo en el ESC seleccionado
 *            5. Vuelta a 0 y mantenimiento continuo de stop
 ******************************************************************************
 */

#include "console.h"
#include "driver_motors.h"

#include <stddef.h>

/* ============================================================== */
/* CONFIGURACION DEL TEST (compile-time)                          */
/* ============================================================== */

/* 1=M1/PA15, 2=M2/PB3, 3=M3/PA2, 4=M4/PA3 */
#ifndef SelectEsc
#define SelectEsc 1
#endif

/* Dejar UNA sola linea activa: */
#define MOTORS_TEST_USE_DSHOT300 1
/* #define MOTORS_TEST_USE_DSHOT600 1 */

#if defined(MOTORS_TEST_USE_DSHOT600) && MOTORS_TEST_USE_DSHOT600
#define MOTORS_TEST_PROTOCOL MOTORS_PROTO_DSHOT600
#elif defined(MOTORS_TEST_USE_DSHOT300) && MOTORS_TEST_USE_DSHOT300
#define MOTORS_TEST_PROTOCOL MOTORS_PROTO_DSHOT300
#else
#error "Define MOTORS_TEST_USE_DSHOT300 or MOTORS_TEST_USE_DSHOT600"
#endif

#if (SelectEsc < 1) || (SelectEsc > 4)
#error "SelectEsc must be 1 (M1), 2 (M2), 3 (M3), or 4 (M4)"
#endif

/* ============================================================== */

#define ESC_ARM_TIME_MS        5000u
#define RAMP_STEP_TIME_MS      2000u
#define FRAME_PERIOD_MS        1u
#define MAX_TEST_THROTTLE_PCT  50u
#define DSHOT_MAX_SAFE_VALUE   1024u

static uint16_t throttle_to_dshot(uint8_t percent)
{
    if (percent == 0u) {
        return 0u;
    }
    if (percent > MAX_TEST_THROTTLE_PCT) {
        percent = MAX_TEST_THROTTLE_PCT;
    }

    const uint32_t span = 2047u - 48u;
    uint16_t value = (uint16_t)(48u + (span * (uint32_t)percent) / 100u);
    if (value > DSHOT_MAX_SAFE_VALUE) {
        value = DSHOT_MAX_SAFE_VALUE;
    }
    return value;
}

static mot_status_t hold_throttle_ms(uint16_t throttle, uint32_t duration_ms)
{
    uint32_t elapsed = 0u;
    while (elapsed < duration_ms) {
        mot_status_t st = motors_set_throttle(throttle);
        if (st != MOT_OK) {
            return st;
        }
        HAL_Delay(FRAME_PERIOD_MS);
        elapsed += FRAME_PERIOD_MS;
    }
    return MOT_OK;
}

static mot_status_t run_ramp_step(uint8_t percent)
{
    uint16_t value = throttle_to_dshot(percent);
    console_printf("Throttle %u%% -> DShot %u (solo %s)\r\n",
                   (unsigned)percent,
                   (unsigned)value,
                   motors_esc_pin_name(SelectEsc));
    return hold_throttle_ms(value, RAMP_STEP_TIME_MS);
}

void test_motors_run(void)
{
    const motors_protocol_t protocol = MOTORS_TEST_PROTOCOL;

    console_banner("Motores ESC DShot (un canal)");

    console_print("========================================\r\n");
    console_print(" ATENCION: SACAR LAS HELICES\r\n");
    console_print(" GND compartido; una sola senal ESC\r\n");
    console_print(" Firmware ANTES de alimentar el ESC\r\n");
    console_print(" (Hobbywing detecta protocolo al boot)\r\n");
    console_print("========================================\r\n");

    console_printf("Protocolo : %s\r\n", motors_protocol_name(protocol));
    console_printf("SelectEsc : %u (%s)\r\n",
                   (unsigned)SelectEsc,
                   motors_esc_pin_name(SelectEsc));
    console_printf("Ramp max  : %u%% (DShot capped @ %u)\r\n",
                   (unsigned)MAX_TEST_THROTTLE_PCT,
                   (unsigned)DSHOT_MAX_SAFE_VALUE);

    mot_status_t st = motors_init(protocol, (uint8_t)SelectEsc);
    if (st != MOT_OK) {
        console_printf("motors_init fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo iniciando TIM2/DMA DShot");
        while (1) {
            HAL_Delay(500u);
        }
    }

    console_printf("Armando %lu ms a throttle=0...\r\n",
                   (unsigned long)ESC_ARM_TIME_MS);
    st = hold_throttle_ms(0u, ESC_ARM_TIME_MS);
    if (st != MOT_OK) {
        console_printf("armado fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo durante armado throttle=0");
        while (1) {
            HAL_Delay(500u);
        }
    }
    console_result(true, "ESC armado (throttle=0)");

    static const uint8_t ramp_up[] = {5u, 10u, 15u, 20u, 25u, 30u, 40u, 50u};
    static const uint8_t ramp_down[] = {40u, 30u, 25u, 20u, 15u, 10u, 5u};

    console_print("Ramp UP\r\n");
    for (size_t i = 0u; i < (sizeof(ramp_up) / sizeof(ramp_up[0])); ++i) {
        st = run_ramp_step(ramp_up[i]);
        if (st != MOT_OK) {
            console_printf("ramp UP fallo (codigo=%d)\r\n", (int)st);
            console_led_fail();
            break;
        }
    }

    if (st == MOT_OK) {
        console_print("Ramp DOWN\r\n");
        for (size_t i = 0u; i < (sizeof(ramp_down) / sizeof(ramp_down[0])); ++i) {
            st = run_ramp_step(ramp_down[i]);
            if (st != MOT_OK) {
                console_printf("ramp DOWN fallo (codigo=%d)\r\n", (int)st);
                console_led_fail();
                break;
            }
        }
    }

    console_print("Vuelta a throttle=0; mantengo stop.\r\n");
    console_result(st == MOT_OK, (st == MOT_OK) ? "secuencia DShot OK"
                                                 : "secuencia DShot con errores");

    while (1) {
        (void)motors_set_throttle(0u);
        HAL_Delay(FRAME_PERIOD_MS);
    }
}
