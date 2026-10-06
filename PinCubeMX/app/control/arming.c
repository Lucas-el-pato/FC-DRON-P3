/**
 ******************************************************************************
 * @file    arming.c
 * @brief   Implementacion del armado / desarmado (ver arming.h).
 ******************************************************************************
 */

#include "arming.h"
#include "mixer.h"
#include "pid.h"
#include "driver_motors.h"
#include "console.h"
#include "timebase.h"
#include "stm32f4xx_hal.h"

/* El ESC aplica un comando DShot de configuracion recien despues de recibirlo
 * ~6 veces seguidas con el motor parado; se mandan 10 como Betaflight.      */
#define ARMING_DSHOT_CMD_REPEAT     10u
#define ARMING_DSHOT_CMD_GAP_US     1000u
#define ARMING_DSHOT_BUSY_TIMEOUT_US 200u

static bool     s_armed = false;
static bool     s_switch_was_low = false;   /* el switch paso por OFF */
static uint32_t s_disable_flags = ARMING_DISABLED_NONE;
static uint32_t s_arm_count = 0u;
static uint32_t s_disarm_count = 0u;

static void arming_send_motor_command(uint16_t cmd, uint8_t esc_mask)
{
    if (esc_mask == 0u) {
        return;
    }

    for (uint8_t i = 0u; i < ARMING_DSHOT_CMD_REPEAT; ++i) {
        const uint32_t t0 = timebase_now();
        while (motors_output_busy() &&
               (timebase_elapsed_us(t0) < ARMING_DSHOT_BUSY_TIMEOUT_US)) {
        }
        (void)motors_send_command4(cmd, esc_mask);
        timebase_delay_us(ARMING_DSHOT_CMD_GAP_US);
    }
}

/* Comandos absolutos (no relativos a la EEPROM del ESC) y sin SAVE: el
 * firmware es la unica fuente del sentido de giro. Se repite en cada armado
 * para cubrir ESC que se alimentaron despues que el FC.                    */
static void arming_apply_motor_directions(void)
{
    const uint8_t reversed = (uint8_t)(MIXER_MOTOR_REVERSED_MASK & 0x0Fu);
    const uint8_t normal = (uint8_t)(~MIXER_MOTOR_REVERSED_MASK & 0x0Fu);

    arming_send_motor_command(MOTORS_DSHOT_CMD_SPIN_DIRECTION_REVERSED, reversed);
    arming_send_motor_command(MOTORS_DSHOT_CMD_SPIN_DIRECTION_NORMAL, normal);
}

void arming_init(void)
{
    s_armed = false;
    s_switch_was_low = false;
    s_disable_flags = ARMING_DISABLED_NONE;
    s_arm_count = 0u;
    s_disarm_count = 0u;
    (void)motors_disarm_all();
    console_led_fail();   /* rojo = desarmado */
}

void arming_disable_set(uint32_t flags)
{
    s_disable_flags |= flags;
}

void arming_disable_clear(uint32_t flags)
{
    s_disable_flags &= ~flags;
}

uint32_t arming_disable_flags(void)
{
    return s_disable_flags;
}

bool arming_is_armed(void)
{
    return s_armed;
}

void arming_disarm(void)
{
    if (s_armed) {
        s_armed = false;
        s_disarm_count++;
    }
    (void)motors_disarm_all();
    pid_reset();
    console_led_fail();
}

void arming_update(bool arm_switch,
                   float throttle,
                   bool link_ok,
                   bool failsafe_disarm,
                   bool failsafe_blocked)
{
    /* Flags dinamicos: se recalculan en cada pasada sin pisar los duros. */
    uint32_t dynamic = ARMING_DISABLED_NONE;

    if (!link_ok) {
        dynamic |= ARMING_DISABLED_RX_LOSS;
    }
    if (throttle > ARMING_THROTTLE_MAX) {
        dynamic |= ARMING_DISABLED_THROTTLE;
    }
    if (failsafe_blocked) {
        dynamic |= ARMING_DISABLED_FAILSAFE;
    }
    if (HAL_GetTick() < ARMING_BOOT_GRACE_MS) {
        dynamic |= ARMING_DISABLED_BOOT_GRACE;
    }

    if (!arm_switch) {
        s_switch_was_low = true;
    }
    if (!s_switch_was_low) {
        dynamic |= ARMING_DISABLED_ARM_SWITCH;
    }

    arming_disable_clear(ARMING_DISABLED_RX_LOSS | ARMING_DISABLED_THROTTLE |
                         ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_BOOT_GRACE |
                         ARMING_DISABLED_ARM_SWITCH);
    arming_disable_set(dynamic);

    /* Desarme: siempre gana sobre el armado. */
    if (s_armed) {
        const bool must_disarm = failsafe_disarm || !arm_switch || !link_ok ||
                                 ((s_disable_flags & (uint32_t)ARMING_DISABLED_NO_GYRO) != 0u) ||
                                 ((s_disable_flags & (uint32_t)ARMING_DISABLED_NO_MOTORS) != 0u);
        if (must_disarm) {
            arming_disarm();
        }
        return;
    }

    /* Armado: switch en alto y cero flags de bloqueo. */
    if (arm_switch && (s_disable_flags == ARMING_DISABLED_NONE)) {
        arming_apply_motor_directions();
        pid_reset();
        s_armed = true;
        s_arm_count++;
        console_led_pass();   /* verde = armado */
    }
}

uint32_t arming_arm_count(void)
{
    return s_arm_count;
}

uint32_t arming_disarm_count(void)
{
    return s_disarm_count;
}
