/**
 ******************************************************************************
 * @file    bal_log.h
 * @brief   Registro de tests del balancin en la microSD, para comparar con el
 *          modelo de Simulink (ejemplo_balancin.slx).
 *
 *          Un test = un "R <deg>" con el balancin armado. Se guardan
 *          BAL_LOG_PRE_MS antes del escalon + BAL_LOG_TEST_MS despues, a
 *          BAL_LOG_RATE_HZ: el escalon cae en t = 0.5 s como en el modelo.
 *
 *          La medicion va entera a RAM (la escritura de la SD es por polling
 *          y frenaria el lazo de 2 kHz). Al terminar el test, bal_log_task()
 *          la vuelca a /BAL/TnnnPxx.CSV de a ~512 B por pasada de la cola
 *          lenta. Mientras vuelca no se puede empezar otro test.
 *
 *          Archivo: lineas "# clave,valor" (ref, throttle, ganancias...) y
 *          despues un CSV con encabezado:
 *            t_s,ref_deg,theta_deg,omega_dps,omega_ref_dps,p,i,d,tau,thr,
 *            vL,vR,m1_dshot,m3_dshot,armed
 *          p..vR en fraccion de motor (0..1), igual que Mixer.m.
 *          /BAL/INDEX.CSV suma una linea por test; /BAL/NEXT.TXT guarda el
 *          numero del proximo.
 ******************************************************************************
 */

#ifndef BALANCER_BAL_LOG_H_
#define BALANCER_BAL_LOG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BAL_LOG_OFF = 0,    /* sin SD: no se graba nada          */
    BAL_LOG_IDLE,       /* listo para un test                */
    BAL_LOG_REC,        /* grabando en RAM                   */
    BAL_LOG_FLUSH       /* volcando el test a la SD          */
} bal_log_state_t;

typedef enum {
    BAL_LOG_START_OK = 0,
    BAL_LOG_START_NO_SD,
    BAL_LOG_START_BUSY  /* volcando (o se acaba de cortar un test) */
} bal_log_start_t;

/* Una muestra, en las unidades del modelo (grados, fraccion de motor). */
typedef struct {
    float    ref_deg;
    float    theta_deg;
    float    omega_dps;
    float    omega_ref_dps;
    float    p;
    float    i;
    float    d;
    float    tau;
    float    thr;
    float    vl;
    float    vr;
    uint16_t m1_dshot;
    uint16_t m3_dshot;
    bool     armed;
} bal_log_input_t;

/* sd_ok: resultado de sd_mount(). Lee /BAL/NEXT.TXT para numerar. */
void bal_log_init(bool sd_ok);

/* A BAL_LOG_RATE_HZ desde el lazo de control. Solo copia a RAM. */
void bal_log_sample(const bal_log_input_t *in);

/* ------------------------------------------------------------------------- */
/* Empieza un test. Llamar ANTES de cambiar la referencia: las muestras      */
/* previas (pre-trigger) tienen que tener la referencia vieja.               */
/* Si habia un test grabando, lo corta y devuelve BUSY (hay que volcarlo).   */
/* ------------------------------------------------------------------------- */
bal_log_start_t bal_log_start(float ref_deg, float ref_prev_deg,
                              float thr, float trim_deg, uint16_t vbat_cv);

/* Corta el test en curso (desarme). Lo grabado se vuelca igual. */
void bal_log_stop(void);

/* Cola lenta: vuelca a la SD un trozo por llamada. */
void bal_log_task(void);

bal_log_state_t bal_log_state(void);
const char *bal_log_state_name(void);
uint16_t bal_log_test_number(void);   /* test en curso o el proximo */
uint32_t bal_log_errors(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCER_BAL_LOG_H_ */
