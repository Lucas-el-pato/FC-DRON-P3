/**
 ******************************************************************************
 * @file    test_motor_dir.c
 * @brief   Configura el sentido de giro de los ESC por comandos DShot y lo
 *          guarda en la EEPROM del ESC (AM32 / BLHeli_32).
 *
 *          !! SIN HELICES !!
 *
 *          Objetivo quad-X Betaflight (props-in, visto desde arriba):
 *            M1 trasero derecho   -> horario
 *            M2 delantero derecho -> antihorario
 *            M3 trasero izquierdo -> antihorario
 *            M4 delantero izq.    -> horario
 *
 *          Partiendo de los 4 en antihorario (direccion normal del ESC),
 *          basta con invertir M1 y M4. Ajustar MOTOR_DIR_* si el punto de
 *          partida es otro.
 *
 *          Secuencia:
 *            1. Flashear y arrancar el FC; recien entonces alimentar los ESC.
 *            2. Armado: throttle 0 en los 4 durante MOTOR_DIR_ARM_MS.
 *            3. Comando 7/8 x10 a cada ESC segun la tabla, y SAVE (12) x10.
 *            4. Prueba: cada motor gira solo a throttle bajo, M1 -> M4, para
 *               confirmar a ojo el sentido.
 *            5. Throttle 0 continuo.
 *
 *          El cambio queda guardado en el ESC: correr este test una sola vez.
 ******************************************************************************
 */

#include "console.h"
#include "driver_motors.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>

typedef enum {
    MOTOR_DIR_KEEP = 0,
    MOTOR_DIR_NORMAL,
    MOTOR_DIR_REVERSED
} motor_dir_t;

/* Indice 0 = M1 .. 3 = M4. */
static const motor_dir_t MOTOR_DIR_TARGET[MOTORS_COUNT] = {
    MOTOR_DIR_REVERSED,  /* M1 -> horario     */
    MOTOR_DIR_KEEP,      /* M2 -> antihorario */
    MOTOR_DIR_KEEP,      /* M3 -> antihorario */
    MOTOR_DIR_REVERSED,  /* M4 -> horario     */
};

#define MOTOR_DIR_ARM_MS          5000u
#define MOTOR_DIR_CMD_REPEAT      10u
#define MOTOR_DIR_FRAME_MS        1u
#define MOTOR_DIR_SAVE_SETTLE_MS  1000u
#define MOTOR_DIR_SPIN_DSHOT      200u   /* ~8 % del rango DShot */
#define MOTOR_DIR_SPIN_MS         3000u
#define MOTOR_DIR_PAUSE_MS        1500u

static const char *const MOTOR_NAME[MOTORS_COUNT] = {
    "M1 trasero derecho",
    "M2 delantero derecho",
    "M3 trasero izquierdo",
    "M4 delantero izq.",
};

static void wait_frame_done(void)
{
    const uint32_t start = HAL_GetTick();
    while (motors_output_busy() && (HAL_GetTick() - start) < 5u) {
    }
}

static void hold_outputs_ms(const uint16_t thr[MOTORS_COUNT], uint32_t duration_ms)
{
    const uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms) {
        wait_frame_done();
        (void)motors_write4(thr, 0u);
        HAL_Delay(MOTOR_DIR_FRAME_MS);
    }
}

static void hold_zero_ms(uint32_t duration_ms)
{
    static const uint16_t zeros[MOTORS_COUNT] = { 0u, 0u, 0u, 0u };
    hold_outputs_ms(zeros, duration_ms);
}

static bool send_command_repeated(uint16_t cmd, uint8_t esc_mask)
{
    for (uint8_t i = 0u; i < MOTOR_DIR_CMD_REPEAT; ++i) {
        wait_frame_done();
        if (motors_send_command4(cmd, esc_mask) != MOT_OK) {
            return false;
        }
        HAL_Delay(MOTOR_DIR_FRAME_MS);
    }
    wait_frame_done();
    return true;
}

static uint8_t mask_for(motor_dir_t dir)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0u; i < MOTORS_COUNT; ++i) {
        if (MOTOR_DIR_TARGET[i] == dir) {
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

static void halt_fail(const char *msg)
{
    console_result(false, msg);
    while (1) {
        hold_zero_ms(500u);
        console_led_fail();
    }
}

void test_motor_dir_run(void)
{
    console_banner("Sentido de giro ESC por DShot (SIN HELICES)");

    console_print("========================================\r\n");
    console_print(" ATENCION: SACAR LAS HELICES\r\n");
    console_print(" Alimentar los ESC DESPUES de arrancar el FC\r\n");
    console_print("========================================\r\n");

    if (motors_init_all(MOTORS_PROTO_DSHOT300) != MOT_OK) {
        halt_fail("motors_init_all");
    }

    console_printf("Armando %lu ms a throttle=0...\r\n",
                   (unsigned long)MOTOR_DIR_ARM_MS);
    hold_zero_ms(MOTOR_DIR_ARM_MS);

    const uint8_t mask_rev = mask_for(MOTOR_DIR_REVERSED);
    const uint8_t mask_nor = mask_for(MOTOR_DIR_NORMAL);

    if (mask_rev != 0u) {
        console_printf("Comando REVERSED (8) -> mascara 0x%X\r\n", (unsigned)mask_rev);
        if (!send_command_repeated(MOTORS_DSHOT_CMD_SPIN_DIRECTION_REVERSED, mask_rev)) {
            halt_fail("envio comando 8");
        }
        hold_zero_ms(100u);
    }
    if (mask_nor != 0u) {
        console_printf("Comando NORMAL (7) -> mascara 0x%X\r\n", (unsigned)mask_nor);
        if (!send_command_repeated(MOTORS_DSHOT_CMD_SPIN_DIRECTION_NORMAL, mask_nor)) {
            halt_fail("envio comando 7");
        }
        hold_zero_ms(100u);
    }

    const uint8_t mask_save = (uint8_t)(mask_rev | mask_nor);
    if (mask_save != 0u) {
        console_printf("SAVE (12) -> mascara 0x%X\r\n", (unsigned)mask_save);
        if (!send_command_repeated(MOTORS_DSHOT_CMD_SAVE_SETTINGS, mask_save)) {
            halt_fail("envio comando 12");
        }
        hold_zero_ms(MOTOR_DIR_SAVE_SETTLE_MS);
    }
    console_result(true, "comandos de direccion enviados y guardados");

    console_print("Prueba de giro: mirar desde ARRIBA.\r\n");
    console_print("Esperado: M1 y M4 horario, M2 y M3 antihorario.\r\n");
    for (uint8_t i = 0u; i < MOTORS_COUNT; ++i) {
        uint16_t thr[MOTORS_COUNT] = { 0u, 0u, 0u, 0u };
        thr[i] = MOTOR_DIR_SPIN_DSHOT;

        console_printf("  %s girando %lu ms...\r\n",
                       MOTOR_NAME[i], (unsigned long)MOTOR_DIR_SPIN_MS);
        hold_outputs_ms(thr, MOTOR_DIR_SPIN_MS);
        hold_zero_ms(MOTOR_DIR_PAUSE_MS);
    }

    console_result(true, "secuencia terminada; throttle=0 continuo");
    while (1) {
        hold_zero_ms(1000u);
    }
}
