/**
 ******************************************************************************
 * @file    test_runner.h
 * @brief   Selector de test por #define.
 *
 *          Descomentar UNA sola macro TEST_SELECT_xxx antes de compilar.
 *          test_runner_run() invoca la funcion test_xxx_run() correspondiente
 *          y nunca retorna (cada test_xxx_run() corre un bucle infinito al
 *          final).
 *
 *          Si no se define ninguna, main() llama fc_run() directamente.
 ******************************************************************************
 */

#ifndef TESTS_INC_TEST_RUNNER_H_
#define TESTS_INC_TEST_RUNNER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================== */
/* SELECCION DE TEST: dejar UNA sola linea descomentada.          */
/* Si ninguna esta activa, main() ejecuta fc_run().               */
/* ============================================================== */
//#define TEST_SELECT_IMU
//#define TEST_SELECT_IMU_DIAG
//#define TEST_SELECT_BARO
//#define TEST_SELECT_MAG
//#define TEST_SELECT_GPS
//#define TEST_SELECT_RC
//#define TEST_SELECT_MOTORS
//#define TEST_SELECT_TELEMETRY
//#define TEST_SELECT_CRSF_TELEM
//#define TEST_SELECT_SD
/* ============================================================== */

/* Cuenta cuantos TEST_SELECT_xxx estan definidos (main usa esto). */
#define TEST_RUNNER_COUNT \
    ( (defined(TEST_SELECT_IMU)      ? 1 : 0) \
    + (defined(TEST_SELECT_IMU_DIAG) ? 1 : 0) \
    + (defined(TEST_SELECT_BARO)     ? 1 : 0) \
    + (defined(TEST_SELECT_MAG)      ? 1 : 0) \
    + (defined(TEST_SELECT_GPS)      ? 1 : 0) \
    + (defined(TEST_SELECT_RC)       ? 1 : 0) \
    + (defined(TEST_SELECT_MOTORS)   ? 1 : 0) \
    + (defined(TEST_SELECT_TELEMETRY) ? 1 : 0) \
    + (defined(TEST_SELECT_CRSF_TELEM) ? 1 : 0) \
    + (defined(TEST_SELECT_SD)         ? 1 : 0) )

/* Punto de entrada del runner; se llama desde main(). No retorna. */
void test_runner_run(void);

/* Prototipos de cada test (definidos en test_xxx.c). */
void test_imu_run(void);
void test_imu_diag_run(void);
void test_baro_run(void);
void test_mag_run(void);
void test_gps_run(void);
void test_rc_run(void);
void test_motors_run(void);
void test_telemetry_run(void);
void test_crsf_telem_run(void);
void test_sd_run(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_TEST_RUNNER_H_ */
