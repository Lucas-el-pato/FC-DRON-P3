/**
 ******************************************************************************
 * @file    test_motors.c
 * @brief   Test de los motores ESC con DShot600 (TIM2 + DMA).
 *
 *          ATENCION: este test envia comandos REALES al ESC.
 *          - SACAR LAS HELICES antes de correrlo.
 *          - Tener el ESC alimentado por su BEC/bateria.
 *          - Conectar solo el motor que se quiere probar (o los 4 si se
 *            quiere ver el barrido en simultaneo).
 *
 *          Secuencia:
 *            1. motors_init() -> envia frames throttle=0 durante 1 s para
 *               armar los ESC.
 *            2. Barrido lento de throttle de 48 a 200 (muy bajo, para que
 *               solo gire si el motor esta libre y sin carga). Cada paso
 *               se mantiene 200 ms.
 *            3. Vuelta a throttle=0.
 *          El test pasa si motors_init y todas las llamadas devuelven MOT_OK.
 ******************************************************************************
 */

#include "console.h"
#include "driver_motors.h"

void test_motors_run(void)
{
    console_banner("Motores ESC DShot600");

    console_print("ATENCION: SACAR LAS HELICES antes de continuar.\r\n");
    console_print("Inicializando motores (armado 1 s a throttle=0)...\r\n");

    mot_status_t st = motors_init();
    if (st != MOT_OK) {
        console_printf("motors_init fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo iniciando TIM2/DMA DShot");
        while (1) { HAL_Delay(500u); }
    }

    console_result(true, "DShot600 armado correctamente");

    /* Barrido suave: 48 -> 200, paso 8, 200 ms cada paso. */
    for (uint16_t t = 48u; t <= 200u; t += 8u) {
        console_printf("Throttle = %u\r\n", (unsigned)t);
        for (uint8_t i = 0u; i < 10u; ++i) {
            st = motors_set_throttle(t, t, t, t);
            if (st != MOT_OK) {
                console_printf("motors_set_throttle fallo (codigo=%d)\r\n", (int)st);
                console_led_fail();
            }
            HAL_Delay(20u);
        }
    }

    console_print("Vuelta a throttle = 0\r\n");
    for (uint8_t i = 0u; i < 50u; ++i) {
        motors_set_throttle(0u, 0u, 0u, 0u);
        HAL_Delay(10u);
    }

    console_print("Test de motores finalizado. Mantengo throttle=0.\r\n");
    while (1) {
        motors_set_throttle(0u, 0u, 0u, 0u);
        HAL_Delay(10u);
    }
}
