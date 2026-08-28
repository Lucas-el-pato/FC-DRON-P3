/**
 ******************************************************************************
 * @file    test_imu.c
 * @brief   Test del IMU LSM6DSV16X: WHO_AM_I + medicion de dt DRDY + lectura.
 ******************************************************************************
 */

#include "console.h"
#include "driver_imu.h"
#include "timebase.h"

#define IMU_DT_NOMINAL_US_X100   12500u   /* 125.00 us @ 8 kHz */
#define IMU_DT_TOLERANCE_PCT     2u
#define IMU_DT_MIN_US_X100       (IMU_DT_NOMINAL_US_X100 * (100u - IMU_DT_TOLERANCE_PCT) / 100u)
#define IMU_DT_MAX_US_X100       (IMU_DT_NOMINAL_US_X100 * (100u + IMU_DT_TOLERANCE_PCT) / 100u)
#define IMU_RATE_MIN_HZ          7840u
#define IMU_RATE_MAX_HZ          8160u
#define IMU_ODR_WINDOW_US        100000u
#define IMU_REPORT_PERIOD_US     500000u

static bool imu_timing_pass(uint32_t dt_avg_x100, uint32_t rate_hz, uint32_t missed)
{
    if (missed != 0u) {
        return false;
    }
    if (dt_avg_x100 < IMU_DT_MIN_US_X100 || dt_avg_x100 > IMU_DT_MAX_US_X100) {
        return false;
    }
    if (rate_hz < IMU_RATE_MIN_HZ || rate_hz > IMU_RATE_MAX_HZ) {
        return false;
    }
    return true;
}

void test_imu_run(void)
{
    /* imu_init() incluye T_BOOT (~50 ms) + WHO_AM_I. El LED va ANTES de
     * console_init() para no esperar DTR USB. */
    imu_status_t st = imu_init();
    if (st != IMU_OK) {
        console_led_fail();
        console_init();
        console_banner("IMU LSM6DSV16X");
        console_printf("imu_init fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo en imu_init");
        while (1) {
            HAL_Delay(500u);
        }
    }

    console_led_pass();
    console_init();

    console_banner("IMU LSM6DSV16X");
    console_printf("timebase_ready=%u  cycles_per_us=%lu\r\n",
                   (unsigned)timebase_ready(),
                   (unsigned long)timebase_cycles_per_us());

    /* Autotest: HAL_Delay(100) medido con DWT. */
    {
        const uint32_t t0 = timebase_now();
        HAL_Delay(100u);
        const uint32_t measured_us = timebase_delta_us(t0, timebase_now());
        console_printf("timebase autotest: HAL_Delay(100) = %lu us (esperado ~100000)\r\n",
                       (unsigned long)measured_us);
    }

    uint8_t who = 0u;
    (void)imu_check_who_am_i(&who);
    console_printf("WHO_AM_I leido = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)who, (unsigned)IMU_WHO_AM_I_VALUE);
    console_result(true, "IMU inicializada correctamente");

    /* Verificar HAODR 8 kHz y contar muestras en 100 ms con timebase. */
    uint8_t haodr = 0u, ctrl1 = 0u, ctrl2 = 0u, int1_ctrl = 0u;
    (void)imu_read_reg(IMU_REG_HAODR_CFG, &haodr);
    (void)imu_read_reg(IMU_REG_CTRL1, &ctrl1);
    (void)imu_read_reg(IMU_REG_CTRL2, &ctrl2);
    (void)imu_read_reg(IMU_REG_INT1_CTRL, &int1_ctrl);
    console_printf("HAODR_CFG = 0x%02X (esperado 0x%02X = SEL 8 kHz)\r\n",
                   (unsigned)haodr, (unsigned)IMU_HAODR_SEL_8KHZ);
    console_printf("CTRL1 = 0x%02X  CTRL2 = 0x%02X (esperado 0x%02X = HAODR+8k)\r\n",
                   (unsigned)ctrl1, (unsigned)ctrl2, (unsigned)IMU_CTRL_HAODR_8KHZ);
    console_printf("INT1_CTRL = 0x%02X (esperado 0x%02X = INT1_DRDY_G)\r\n",
                   (unsigned)int1_ctrl, (unsigned)IMU_INT1_DRDY_G);

    {
        const uint32_t t0_cyc = timebase_now();
        uint32_t n_ok = 0u;

        while (timebase_elapsed_us(t0_cyc) < IMU_ODR_WINDOW_US) {
            if (!imu_gyro_drdy_take()) {
                continue;
            }
            imu_sample_t s = { 0 };
            if (imu_read_sample(&s) == IMU_OK) {
                n_ok++;
            }
        }

        const uint32_t span_cyc = timebase_now() - t0_cyc;
        const uint32_t span_us = timebase_cycles_to_us(span_cyc);
        const uint32_t rate_x10 = (span_us > 0u)
            ? (uint32_t)(((uint64_t)n_ok * 10000000u) / (uint64_t)span_us)
            : 0u;
        const uint32_t dt_avg_x100 = (n_ok > 1u)
            ? timebase_cycles_to_us_x100(span_cyc / (n_ok - 1u))
            : 0u;

        /* Una sola llamada / un solo transmit USB: evita que el propio
         * reporte se coma muestras del ODR que esta midiendo.            */
        console_printf("muestras INT1 en %lu us = %lu (ODR 8 kHz -> ~800)\r\n"
                       "  rate=%lu.%lu Hz  dt_avg~%lu.%02lu us\r\n",
                       (unsigned long)span_us,
                       (unsigned long)n_ok,
                       (unsigned long)(rate_x10 / 10u),
                       (unsigned long)(rate_x10 % 10u),
                       (unsigned long)(dt_avg_x100 / 100u),
                       (unsigned long)(dt_avg_x100 % 100u));
    }

    /* Bucle continuo: lee en cada DRDY, reporta dt/jitter cada 500 ms. */
    uint32_t t_report = timebase_now();
    imu_sample_t last = { 0 };

    while (1) {
        if (imu_gyro_drdy_take()) {
            imu_sample_t s = { 0 };
            if (imu_read_sample(&s) == IMU_OK) {
                last = s;
            }
        }

        if (timebase_elapsed_us(t_report) >= IMU_REPORT_PERIOD_US) {
            t_report = timebase_now();

            imu_drdy_stats_t stats;
            imu_gyro_drdy_stats_take(&stats);

            const uint32_t dt_avg_x100 = timebase_cycles_to_us_x100(stats.dt_avg_cyc);
            const uint32_t dt_min_x100 = timebase_cycles_to_us_x100(stats.dt_min_cyc);
            const uint32_t dt_max_x100 = timebase_cycles_to_us_x100(stats.dt_max_cyc);
            const uint32_t jitter_x100 = (stats.intervals > 0u)
                ? (dt_max_x100 - dt_min_x100)
                : 0u;
            const uint32_t span_us = timebase_cycles_to_us(stats.window_cyc);
            const uint32_t rate_hz = (span_us > 0u)
                ? (uint32_t)(((uint64_t)stats.edges * 1000000u) / (uint64_t)span_us)
                : 0u;
            const uint32_t missed = imu_gyro_drdy_missed_estimate(stats.late_cyc,
                                                                  stats.late_events);
            const bool pass = imu_timing_pass(dt_avg_x100, rate_hz, missed);

            /* Una sola llamada / un solo transmit USB por reporte: con
             * DRDY latched, cualquier stall de impresion mayor a 125 us
             * se ve como "missed", asi que fragmentar en varias llamadas
             * CDC (cada una con reintento propio) infla missed de forma
             * artificial. Un solo console_printf() = un solo paquete USB. */
            console_printf("dt=%lu.%02lu us  min=%lu.%02lu us  max=%lu.%02lu us  "
                           "jitter=%lu.%02lu us  rate=%lu Hz  late=%lu missed=%lu  %s\r\n"
                           "GYRO x=%6d y=%6d z=%6d | ACCEL x=%6d y=%6d z=%6d\r\n",
                           (unsigned long)(dt_avg_x100 / 100u), (unsigned long)(dt_avg_x100 % 100u),
                           (unsigned long)(dt_min_x100 / 100u), (unsigned long)(dt_min_x100 % 100u),
                           (unsigned long)(dt_max_x100 / 100u), (unsigned long)(dt_max_x100 % 100u),
                           (unsigned long)(jitter_x100 / 100u), (unsigned long)(jitter_x100 % 100u),
                           (unsigned long)rate_hz,
                           (unsigned long)stats.late_events,
                           (unsigned long)missed,
                           pass ? "[PASS]" : "[FAIL]",
                           last.gx, last.gy, last.gz, last.ax, last.ay, last.az);
            console_led_set(pass);
        }
    }
}
