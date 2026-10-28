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
 *          10 Hz en total, alternando dos paginas (cada una a 5 Hz).
 *          Payload BIG-endian. Los u16 saturan salvo que diga "16 bits bajos".
 *
 *          Pagina 0 - estado de vuelo (FC_TELEM_DEBUG_LEN_P0 bytes):
 *            off tipo  campo                         escala
 *            0   u8    version (FC_TELEM_DEBUG_VERSION)
 *            1   u8    pagina = 0
 *            2   u8    flags  b0 armado  b1 failsafe  b2 link RC
 *                             b3 calibrado  b4 PID on  b5 baro  b6 mag
 *                             b7 telemetria ESC valida
 *            3   u8    arming_disable_flags (bits 0..7)
 *            4   u8    failsafe_state_t
 *            5   i16   stick roll / pitch / yaw     x1000 (-1..+1)
 *            11  u16   throttle                      x1000 (0..1)
 *            13  i16   setpoint rate roll/pitch/yaw  x10 °/s
 *            19  i16   gyro filtrado roll/pitch/yaw  x10 °/s
 *            25  i16   pid_out roll/pitch/yaw        x1000
 *            31  u16   motor CH1..CH4 (M4,M3,M2,M1)  DShot (0, 48..2047)
 *            39  u16   tiempo maximo del PID         us (ultima ventana)
 *            41  u16   edad del ultimo frame RC      ms
 *            43  u16   errores de gyro               (16 bits bajos)
 *            45  u16   frames de motores perdidos    (16 bits bajos)
 *
 *          Pagina 1 - lazo / scheduler (FC_TELEM_DEBUG_LEN_P1 bytes).
 *          Ventana = tiempo desde la pagina 1 anterior (~200 ms):
 *            off tipo  campo
 *            0   u8    version
 *            1   u8    pagina = 1
 *            2   u16   duracion de la ventana        ms
 *            4   u16   cadenas realtime (muestras de gyro procesadas)
 *            6   u16   ejecuciones del PID
 *            8   u16   cadena realtime max           us (gyro+filtro+PID+RX)
 *            10  u16   PID max                       us
 *            12  u16   intervalo max entre PIDs      us (nominal 500)
 *            14  u16   tarea lenta max               us
 *            16  u16   tareas lentas ejecutadas
 *            18  u16   de esas, forzadas (no entraban antes del PID)
 *            20  u8    indice de la tarea lenta max  (0xFF = ninguna)
 *            21  u16   flancos INT1 del gyro
 *            23  u16   intervalos INT1 tarde (> 1.5x nominal)
 *            25  u16   muestras de gyro perdidas (estimadas)
 *            27  u16   drdy_count - gyro_samples     (desde boot, 16 bits bajos)
 *            29  u16   IMU overrun        (reservado Fase 2, hoy 0)
 *            31  u16   IMU ring overflow  (reservado Fase 2, hoy 0)
 *            33  u16   IMU errores SPI    (reservado Fase 2, hoy 0)
 *            35  u16   muestras descartadas por tope (reservado Fase 2, hoy 0)
 *
 *          Version 1 (firmware anterior): solo la pagina 0, sin el byte de
 *          pagina (todos los offsets un byte antes) y 46 bytes.
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

#define FC_TELEM_DEBUG_VERSION  2u
#define FC_TELEM_DEBUG_LEN_P0   47u
#define FC_TELEM_DEBUG_LEN_P1   37u

/* Periodos de las tareas (us). La radio solo reenvia lo que entra en el
 * ratio de telemetria de ELRS; mandar mas rapido no rompe nada (ELRS se
 * queda con el ultimo de cada tipo), pero tampoco llega mas.            */
#define FC_TELEM_ATT_PERIOD_US    50000u    /* 20 Hz */
#define FC_TELEM_BARO_PERIOD_US  200000u    /*  5 Hz */
#define FC_TELEM_BATT_PERIOD_US  500000u    /*  2 Hz */
#define FC_TELEM_MODE_PERIOD_US  500000u    /*  2 Hz */
#define FC_TELEM_ESC_PERIOD_US   500000u    /*  2 Hz */
#ifndef FC_TELEM_DEBUG_PERIOD_US
#define FC_TELEM_DEBUG_PERIOD_US 100000u    /* 10 Hz, paginas alternadas */
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
