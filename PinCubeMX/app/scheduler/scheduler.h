/**
 ******************************************************************************
 * @file    scheduler.h
 * @brief   Scheduler cooperativo estilo Betaflight (scheduler/scheduler.c),
 *          adaptado a este FC. Ver docs/Diagrama_logica_Betaflight.mmd.
 *
 *          Dos mitades:
 *
 *          1. Cadena REALTIME, disparada por el data-ready del gyro (INT1 en
 *             PC2 / EXTI2, ya implementado en driver_imu.c):
 *               TASK_GYRO   -> cada muestra (8 kHz)
 *               TASK_FILTER -> cada filter_denom muestras
 *               TASK_PID    -> cada pid_denom muestras (2 kHz con denom = 4)
 *               rx_check    -> cada pasada
 *               failsafe    -> cada FAILSAFE_PERIOD_MS
 *
 *          2. Cola NO-REALTIME: una sola tarea por pasada, elegida por
 *             prioridad dinamica
 *               prio = 1 + static_prio * (edad / periodo)
 *             y ejecutada solo si entra en el tiempo que queda hasta el
 *             proximo TASK_PID (estimacion + margen de guarda). Una tarea que
 *             pierde SCHED_MAX_DEFER selecciones seguidas se ejecuta igual,
 *             para que nada quede en inanicion (el "defer 1 de 8" del diagrama).
 *
 *          El scheduler no conoce a las tareas: fc_tasks.c arma la tabla y la
 *          registra. Asi app/scheduler no depende de app/fc.
 ******************************************************************************
 */

#ifndef SCHEDULER_SCHEDULER_H_
#define SCHEDULER_SCHEDULER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Selecciones perdidas seguidas antes de forzar la ejecucion de una tarea. */
#define SCHED_MAX_DEFER        8u

/* Margen que se reserva antes del proximo TASK_PID, en microsegundos. */
#define SCHED_GUARD_US         30u

#define SCHED_MAX_TASKS        16u

typedef void (*sched_task_fn)(void);

typedef struct {
    /* Configuracion (la llena fc_tasks.c). */
    const char    *name;
    sched_task_fn  fn;
    uint32_t       period_us;
    uint8_t        static_prio;   /* 1 = LOW ... 5 = HIGH */

    /* Estado interno del scheduler (no tocar desde afuera). */
    uint32_t       last_exec_cyc;
    uint32_t       est_us;        /* duracion anticipada */
    uint32_t       max_us;        /* peor caso observado */
    uint32_t       runs;
    uint32_t       deferred;
} sched_task_t;

/* Etapas realtime + divisores del lazo de gyro. */
typedef struct {
    sched_task_fn gyro_stage;      /* obligatoria si hay gyro */
    sched_task_fn filter_stage;    /* puede ser NULL */
    sched_task_fn pid_stage;       /* obligatoria si hay gyro */
    sched_task_fn rx_check;        /* cada pasada del lazo, puede ser NULL */
    sched_task_fn failsafe_check;  /* cada failsafe_period_ms, puede ser NULL */
    uint8_t       filter_denom;    /* >= 1 */
    uint8_t       pid_denom;       /* >= 1 */
    uint32_t      failsafe_period_ms;
    uint32_t      pid_period_us;   /* periodo nominal de TASK_PID */
    bool          gyro_enabled;    /* false: solo corre la cola lenta */
} sched_realtime_t;

/* Registra la configuracion. tasks debe vivir mientras corra el scheduler. */
void scheduler_init(const sched_realtime_t *rt, sched_task_t *tasks, uint8_t count);

/* Bucle principal. No retorna. */
void scheduler_run(void);

/* Una sola pasada del bucle (util para tests de banco). */
void scheduler_step(void);

/* Diagnostico. */
uint32_t scheduler_pid_iterations(void);
uint32_t scheduler_gyro_samples(void);
uint32_t scheduler_pid_max_us(void);
uint32_t scheduler_task_runs(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_SCHEDULER_H_ */
