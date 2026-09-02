/**
 ******************************************************************************
 * @file    scheduler.c
 * @brief   Implementacion del scheduler cooperativo (ver scheduler.h).
 ******************************************************************************
 */

#include "scheduler.h"
#include "driver_imu.h"
#include "timebase.h"
#include "stm32f4xx_hal.h"

static sched_realtime_t s_rt;
static sched_task_t    *s_tasks = 0;
static uint8_t          s_task_count = 0u;

static uint32_t s_gyro_counter = 0u;      /* muestras de gyro (pidUpdateCounter) */
static uint32_t s_pid_iterations = 0u;
static uint32_t s_pid_max_us = 0u;
static uint32_t s_last_pid_cyc = 0u;
static uint32_t s_last_failsafe_ms = 0u;

/* Microsegundos que faltan para el proximo TASK_PID (0 si ya vencio). */
static uint32_t scheduler_time_to_pid_us(void)
{
    if (!s_rt.gyro_enabled || s_rt.pid_period_us == 0u) {
        return UINT32_MAX;
    }

    const uint32_t elapsed_us = timebase_elapsed_us(s_last_pid_cyc);
    if (elapsed_us >= s_rt.pid_period_us) {
        return 0u;
    }
    return s_rt.pid_period_us - elapsed_us;
}

/* Prioridad dinamica: 1 + static * (edad / periodo), en enteros x100. */
static uint32_t scheduler_task_priority(const sched_task_t *task, uint32_t now_cyc)
{
    if (task->fn == 0 || task->period_us == 0u) {
        return 0u;
    }

    const uint32_t age_us = timebase_delta_us(task->last_exec_cyc, now_cyc);
    if (age_us < task->period_us) {
        return 0u;   /* todavia no le toca */
    }

    /* x100 para no perder resolucion sin usar float en el lazo. */
    const uint32_t age_ratio = (age_us * 100u) / task->period_us;
    return 100u + (uint32_t)task->static_prio * age_ratio;
}

static void scheduler_execute_task(sched_task_t *task)
{
    const uint32_t t0 = timebase_now();
    task->fn();
    const uint32_t dur_us = timebase_elapsed_us(t0);

    task->last_exec_cyc = t0;
    task->runs++;
    task->deferred = 0u;

    if (dur_us > task->max_us) {
        task->max_us = dur_us;
    }
    /* Media movil simple (peso 1/8) como estimacion anticipada. */
    task->est_us = ((task->est_us * 7u) + dur_us) / 8u;
    if (task->est_us == 0u) {
        task->est_us = 1u;
    }
}

/* Cadena realtime: se corre completa cuando hay muestra nueva de gyro. */
static void scheduler_realtime_chain(void)
{
    if (s_rt.gyro_stage != 0) {
        s_rt.gyro_stage();
    }

    s_gyro_counter++;

    if ((s_rt.filter_stage != 0) &&
        ((s_gyro_counter % (uint32_t)s_rt.filter_denom) == 0u)) {
        s_rt.filter_stage();
    }

    if ((s_rt.pid_stage != 0) &&
        ((s_gyro_counter % (uint32_t)s_rt.pid_denom) == 0u)) {
        const uint32_t t0 = timebase_now();
        s_rt.pid_stage();
        const uint32_t dur_us = timebase_elapsed_us(t0);
        if (dur_us > s_pid_max_us) {
            s_pid_max_us = dur_us;
        }
        s_last_pid_cyc = t0;
        s_pid_iterations++;
    }

    if (s_rt.rx_check != 0) {
        s_rt.rx_check();
    }
}

void scheduler_init(const sched_realtime_t *rt, sched_task_t *tasks, uint8_t count)
{
    if (rt == 0) {
        return;
    }

    s_rt = *rt;
    if (s_rt.filter_denom == 0u) {
        s_rt.filter_denom = 1u;
    }
    if (s_rt.pid_denom == 0u) {
        s_rt.pid_denom = 1u;
    }
    if (s_rt.failsafe_period_ms == 0u) {
        s_rt.failsafe_period_ms = 10u;
    }

    s_tasks = tasks;
    s_task_count = (count > SCHED_MAX_TASKS) ? SCHED_MAX_TASKS : count;

    const uint32_t now = timebase_now();
    for (uint8_t i = 0u; i < s_task_count; ++i) {
        s_tasks[i].last_exec_cyc = now;
        s_tasks[i].est_us = 10u;
        s_tasks[i].max_us = 0u;
        s_tasks[i].runs = 0u;
        s_tasks[i].deferred = 0u;
    }

    s_gyro_counter = 0u;
    s_pid_iterations = 0u;
    s_pid_max_us = 0u;
    s_last_pid_cyc = now;
    s_last_failsafe_ms = HAL_GetTick();
}

void scheduler_step(void)
{
    /* 1. Cadena realtime si hay muestra nueva de gyro. */
    if (s_rt.gyro_enabled && imu_gyro_drdy_take()) {
        scheduler_realtime_chain();
    }

    /* 2. Failsafe: reloj propio de 10 ms, independiente del gyro. */
    if (s_rt.failsafe_check != 0) {
        const uint32_t now_ms = HAL_GetTick();
        if ((now_ms - s_last_failsafe_ms) >= s_rt.failsafe_period_ms) {
            s_last_failsafe_ms = now_ms;
            s_rt.failsafe_check();
        }
    }

    if (s_tasks == 0 || s_task_count == 0u) {
        return;
    }

    /* 3. Cola no-realtime: se elige UNA tarea por prioridad dinamica. */
    const uint32_t now_cyc = timebase_now();
    sched_task_t *selected = 0;
    uint32_t best_prio = 0u;

    for (uint8_t i = 0u; i < s_task_count; ++i) {
        const uint32_t prio = scheduler_task_priority(&s_tasks[i], now_cyc);
        if (prio > best_prio) {
            best_prio = prio;
            selected = &s_tasks[i];
        }
    }

    if (selected == 0) {
        return;
    }

    /* 4. Solo se ejecuta si entra antes del proximo PID, salvo que ya haya
     * perdido demasiadas selecciones seguidas (evita inanicion).          */
    const uint32_t remaining_us = scheduler_time_to_pid_us();
    const bool fits = (remaining_us > (selected->est_us + SCHED_GUARD_US));
    const bool forced = (selected->deferred >= SCHED_MAX_DEFER);

    if (fits || forced) {
        scheduler_execute_task(selected);
    } else {
        selected->deferred++;
    }
}

void scheduler_run(void)
{
    while (1) {
        scheduler_step();
    }
}

uint32_t scheduler_pid_iterations(void)
{
    return s_pid_iterations;
}

uint32_t scheduler_gyro_samples(void)
{
    return s_gyro_counter;
}

uint32_t scheduler_pid_max_us(void)
{
    return s_pid_max_us;
}

uint32_t scheduler_task_runs(uint8_t index)
{
    return (index < s_task_count) ? s_tasks[index].runs : 0u;
}
