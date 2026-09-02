/**
 ******************************************************************************
 * @file    test_fc_rc_motors.c
 * @brief   Bring-up del camino RC -> arming -> mixer -> 4 motores DShot,
 *          SIN gyro y SIN PID.
 *
 *          Es el paso previo a habilitar fc_run() completo: verifica que
 *          - llegan canales del PocketMaster (CRSF por UART4),
 *          - el armado respeta throttle bajo + AUX1 + ventana de boot,
 *          - los 4 ESC reciben frames DShot300 y siguen al stick de throttle,
 *          - al apagar la radio el failsafe desarma en menos de 1 s.
 *
 *          !! SIN HELICES !!  Bateria del ESC con la helice desmontada.
 *
 *          Salida por USB CDC a 2 Hz: canales crudos, estado y salida DShot.
 ******************************************************************************
 */

#include "console.h"
#include "timebase.h"

#include "fc_rc.h"
#include "fc_state.h"

#include "arming.h"
#include "failsafe.h"
#include "mixer.h"
#include "pid.h"

#include "driver_motors.h"
#include "stm32f4xx_hal.h"

#define TEST_OUTPUT_PERIOD_MS   2u     /* 500 Hz de frames DShot */
#define TEST_FAILSAFE_PERIOD_MS FAILSAFE_PERIOD_MS
#define TEST_LOG_PERIOD_MS      500u

void test_fc_rc_motors_run(void)
{
    console_banner("FC: RC CRSF -> 4 motores DShot (sin PID, SIN HELICES)");

    fc_state_init();
    arming_init();
    failsafe_init();
    mixer_init();
    pid_init(1.0f / 2000.0f);
    fc_rc_init();

    if (motors_init_all(MOTORS_PROTO_DSHOT300) != MOT_OK) {
        arming_disable_set(ARMING_DISABLED_NO_MOTORS);
        console_result(false, "motors_init_all");
        while (1) {
            console_led_fail();
            HAL_Delay(500u);
        }
    }
    console_result(true, "Motores DShot300 en M1..M4");
    console_print("Esperando canales CRSF. Armar: throttle abajo + AUX1 arriba.\r\n");

    fc_state_t *st = fc_state();
    uint32_t t_out = HAL_GetTick();
    uint32_t t_fs = t_out;
    uint32_t t_log = t_out;

    while (1) {
        /* 1. RC (no bloqueante). */
        if (fc_rc_poll()) {
            st->rx_frames++;
        }

        const uint32_t now = HAL_GetTick();

        /* 2. Failsafe cada 10 ms. */
        if ((now - t_fs) >= TEST_FAILSAFE_PERIOD_MS) {
            t_fs = now;
            fc_rc_update_link();
            if (failsafe_update(fc_rc_get()->link_ok, arming_is_armed())) {
                arming_disarm();
            }
        }

        /* 3. Salida a los motores cada 2 ms (sin PID: pid_axis = NULL). */
        if ((now - t_out) >= TEST_OUTPUT_PERIOD_MS) {
            t_out = now;

            const fc_rc_t *rc = fc_rc_get();
            arming_update(rc->arm_switch, rc->throttle, rc->link_ok,
                          false, failsafe_active());

            mixer_run(rc->throttle, 0, arming_is_armed(), st->motor);
            if (motors_write4(st->motor, 0u) == MOT_OK) {
                st->motor_frames++;
            } else {
                st->motor_drops++;
            }
        }

        /* 4. Log 2 Hz. */
        if ((now - t_log) >= TEST_LOG_PERIOD_MS) {
            t_log = now;
            const fc_rc_t *rc = fc_rc_get();
            console_printf("CH thr=%4u roll=%4u pitch=%4u yaw=%4u aux1=%4u | "
                           "link=%u age=%lums armado=%u flags=0x%02lX fs=%s | "
                           "M %4u %4u %4u %4u | frames=%lu drops=%lu\r\n",
                           (unsigned)rc->raw[FC_RC_CH_THROTTLE],
                           (unsigned)rc->raw[FC_RC_CH_ROLL],
                           (unsigned)rc->raw[FC_RC_CH_PITCH],
                           (unsigned)rc->raw[FC_RC_CH_YAW],
                           (unsigned)rc->raw[FC_RC_CH_ARM],
                           (unsigned)rc->link_ok,
                           (unsigned long)rc->age_ms,
                           (unsigned)arming_is_armed(),
                           (unsigned long)arming_disable_flags(),
                           failsafe_state_name(failsafe_get_state()),
                           (unsigned)st->motor[0], (unsigned)st->motor[1],
                           (unsigned)st->motor[2], (unsigned)st->motor[3],
                           (unsigned long)st->motor_frames,
                           (unsigned long)st->motor_drops);
        }
    }
}
