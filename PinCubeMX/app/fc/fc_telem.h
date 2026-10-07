/**
 ******************************************************************************
 * @file    fc_telem.h
 * @brief   Telemetria CRSF hacia la radio (y de ahi a la estacion terrena por
 *          el USB de la radio en modo Telem Mirror).
 *
 *          Frames estandar (los muestra tambien EdgeTX):
 *            0x1E Attitude        20 Hz
 *            0x09 Baro altitude    5 Hz   (si hay baro)
 *            0x07 Vario            5 Hz   (si hay baro)
 *            0x08 Battery          2 Hz   (desde la telemetria KISS del ESC)
 *            0x21 Flight mode      2 Hz
 *            0x0C RPM / 0x0D Temp  2 Hz   (ESC; temp tambien del baro)
 *
 *          Frame propio CRSF_TYPE_FC_DEBUG (0x7F, extendido, dest = radio),
 *          10 Hz. Payload BIG-endian, FC_TELEM_DEBUG_LEN bytes:
 *
 *            off tipo  campo                         escala
 *            0   u8    version (FC_TELEM_DEBUG_VERSION)
 *            1   u8    flags  b0 armado  b1 failsafe  b2 link RC
 *                             b3 calibrado  b4 PID on  b5 baro  b6 mag
 *                             b7 telemetria ESC valida
 *            2   u8    arming_disable_flags (bits 0..7)
 *            3   u8    failsafe_state_t
 *            4   i16   stick roll / pitch / yaw     x1000 (-1..+1)
 *            10  u16   throttle                      x1000 (0..1)
 *            12  i16   setpoint rate roll/pitch/yaw  x10 °/s
 *            18  i16   gyro filtrado roll/pitch/yaw  x10 °/s
 *            24  i16   pid_out roll/pitch/yaw        x1000
 *            30  u16   motor M1..M4                  DShot (0, 48..2047)
 *            38  u16   tiempo maximo del PID         us
 *            40  u16   edad del ultimo frame RC      ms (satura)
 *            42  u16   errores de gyro               (16 bits bajos)
 *            44  u16   frames de motores perdidos    (16 bits bajos)
 *
 *          El decodificador del lado PC esta en tools/webpage/crsf_link.py;
 *          si se cambia este formato, subir FC_TELEM_DEBUG_VERSION y
 *          actualizar ese archivo.
 ******************************************************************************
 */

#ifndef FC_FC_TELEM_H_
#define FC_FC_TELEM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FC_TELEM_DEBUG_VERSION  1u
#define FC_TELEM_DEBUG_LEN      46u

/* Periodos de las tareas (us). La radio solo reenvia lo que entra en el
 * ratio de telemetria de ELRS; mandar mas rapido no rompe nada (ELRS se
 * queda con el ultimo de cada tipo), pero tampoco llega mas.            */
#define FC_TELEM_ATT_PERIOD_US    50000u    /* 20 Hz */
#define FC_TELEM_BARO_PERIOD_US  200000u    /*  5 Hz */
#define FC_TELEM_BATT_PERIOD_US  500000u    /*  2 Hz */
#define FC_TELEM_MODE_PERIOD_US  500000u    /*  2 Hz */
#define FC_TELEM_ESC_PERIOD_US   500000u    /*  2 Hz */
#ifndef FC_TELEM_DEBUG_PERIOD_US
#define FC_TELEM_DEBUG_PERIOD_US 100000u    /* 10 Hz */
#endif

void fc_telem_attitude(void);
void fc_telem_baro(void);
void fc_telem_battery(void);
void fc_telem_flight_mode(void);
void fc_telem_esc(void);
void fc_telem_debug(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_FC_TELEM_H_ */
