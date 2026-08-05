/**
 ******************************************************************************
 * @file    test_telemetry.h
 * @brief   Combined IMU + baro + mag telemetry stream for the web dashboard.
 ******************************************************************************
 */

#ifndef TESTS_INC_TEST_TELEMETRY_H_
#define TESTS_INC_TEST_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Initializes all three sensors and streams newline-delimited JSON over USB
 * CDC. Never returns. */
void test_telemetry_run(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_TEST_TELEMETRY_H_ */
