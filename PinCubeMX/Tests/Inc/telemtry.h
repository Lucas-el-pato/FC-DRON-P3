/**
 ******************************************************************************
 * @file    telemtry.h
 * @brief   Receptor KISS/AM32 ESC serial telemetry (USART6 RX @ 115200).
 *
 *          Protocolo (10 bytes), compatible con AM32 kiss_telemetry:
 *            [0]     temperature  int8  °C
 *            [1..2]  voltage      uint16 big-endian, centivolts
 *            [3..4]  current      uint16 big-endian, centiamps
 *            [5..6]  consumption  uint16 big-endian, mAh
 *            [7..8]  eRPM/100     uint16 big-endian
 *            [9]     CRC8 (KISS)
 *
 *          Cableado:
 *            ESC T / TELEMETRY  ->  STM32 PC7 (USART6_RX)
 *            ESC GND            ->  STM32 GND
 *
 *          Solicitud: bit de telemetria DShot (motors_set_throttle_telem)
 *          o Auto Telemetry 30 ms en AM32 Config Tool.
 ******************************************************************************
 */

#ifndef TESTS_INC_TELEMTRY_H_
#define TESTS_INC_TELEMTRY_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TELEMTRY_MOTOR_POLES
#define TELEMTRY_MOTOR_POLES 14u
#endif

#define TELEMTRY_FRAME_LEN 10u

typedef enum {
    TELEMTRY_OK = 0,
    TELEMTRY_ERR_PARAM,
    TELEMTRY_ERR_UART,
    TELEMTRY_ERR_CRC,
    TELEMTRY_ERR_TIMEOUT,
    TELEMTRY_ERR_NONE   /* poll: sin frame completo aun */
} telemtry_status_t;

typedef struct {
    int8_t   temperature_c;   /* °C */
    uint16_t voltage_cv;      /* centivolts (100 = 1.00 V) */
    uint16_t current_ca;      /* centiamps  (100 = 1.00 A) */
    uint16_t consumption_mah; /* mAh */
    uint16_t erpm_div100;     /* packet eRPM / 100 */
    uint32_t erpm;            /* electrical RPM */
    uint32_t rpm;             /* mechanical RPM (poles/2) */
    bool     valid;
} telemtry_data_t;

/* Globals for CubeIDE Expressions / Live Expressions (always in scope). */
extern uint8_t         g_telemRxBuf[TELEMTRY_FRAME_LEN];
extern uint8_t         g_telemRxLen;
extern telemtry_data_t g_telemLast;
extern int32_t         g_telemLastStatus; /* telemtry_status_t of last poll/read */
/* Debug probes: global/volatile so CubeIDE can evaluate them in any scope. */
extern volatile uint32_t g_telemDebugMagic;
extern volatile uint32_t g_telemPollCount;
extern volatile uint32_t g_telemByteCount;
extern volatile uint32_t g_telemValidCount;

/* Limpia errores UART6 y vacia RX. */
void telemtry_init(void);

/* CRC8 KISS (misma formula que AM32 / KISS PDF). */
uint8_t telemtry_crc8(const uint8_t *buf, uint8_t len);

/* No bloqueante: consume bytes disponibles; TELEMTRY_OK si hay frame CRC OK. */
telemtry_status_t telemtry_poll(telemtry_data_t *out);

/* Espera hasta frame valido o timeout. */
telemtry_status_t telemtry_read(telemtry_data_t *out, uint32_t timeout_ms);

/* Helpers de conversion. */
static inline float telemtry_voltage_v(const telemtry_data_t *d)
{
    return (d != NULL) ? ((float)d->voltage_cv * 0.01f) : 0.0f;
}

static inline float telemtry_current_a(const telemtry_data_t *d)
{
    return (d != NULL) ? ((float)d->current_ca * 0.01f) : 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_TELEMTRY_H_ */
