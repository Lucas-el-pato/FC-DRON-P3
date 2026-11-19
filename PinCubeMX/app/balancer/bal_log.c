/**
 ******************************************************************************
 * @file    bal_log.c
 * @brief   Registro de tests del balancin en la SD (ver bal_log.h).
 ******************************************************************************
 */

#include "bal_log.h"
#include "bal_config.h"
#include "driver_sd.h"

#include <stdio.h>
#include <string.h>

#define BAL_LOG_PRE_N    ((BAL_LOG_RATE_HZ * BAL_LOG_PRE_MS) / 1000u)
#define BAL_LOG_POST_N   ((BAL_LOG_RATE_HZ * BAL_LOG_TEST_MS) / 1000u)
#define BAL_LOG_MAX_N    (BAL_LOG_PRE_N + BAL_LOG_POST_N)
#define BAL_LOG_DT_US    (1000000u / BAL_LOG_RATE_HZ)

#define BAL_LOG_CHUNK    512u    /* bytes por escritura (un sector) */
#define BAL_LOG_ROW_MAX  128u
#define BAL_LOG_PATH_MAX 24u

#define BAL_LOG_NEXT_FILE   BAL_LOG_DIR "/NEXT.TXT"
#define BAL_LOG_INDEX_FILE  BAL_LOG_DIR "/INDEX.CSV"

/* Escalas de punto fijo (nano.specs no imprime floats). */
#define Q_DEG     100.0f     /* centesimas de grado      */
#define Q_DPS     10.0f      /* decimas de grado/s       */
#define Q_FRAC    10000.0f   /* fraccion de motor x 1e4  */

typedef struct {
    int16_t  ref;        /* x Q_DEG  */
    int16_t  theta;      /* x Q_DEG  */
    int16_t  omega;      /* x Q_DPS  */
    int16_t  omega_ref;  /* x Q_DPS  */
    int16_t  p;          /* x Q_FRAC */
    int16_t  i;
    int16_t  d;
    int16_t  tau;
    int16_t  thr;
    int16_t  vl;
    int16_t  vr;
    uint16_t m1;
    uint16_t m3;
    uint8_t  armed;
} bal_log_rec_t;

typedef struct {
    uint16_t test;
    float    ref_deg;
    float    ref_prev_deg;
    float    thr;
    float    trim_deg;
    uint16_t vbat_cv;
    uint32_t step_index;   /* muestra en la que cambia la referencia */
    char     end;          /* 'C' completo, 'R' cortado por R, 'D' desarme */
} bal_log_meta_t;

static bal_log_state_t s_state = BAL_LOG_OFF;
static uint16_t s_next_test = 1u;
static uint32_t s_errors = 0u;

/* Pre-trigger: ultimas BAL_LOG_PRE_N muestras, corre siempre. */
static bal_log_rec_t s_pre[BAL_LOG_PRE_N];
static uint32_t s_pre_head = 0u;
static uint32_t s_pre_count = 0u;

/* Test completo en RAM (~59 KB). */
static bal_log_rec_t s_buf[BAL_LOG_MAX_N];
static uint32_t s_n = 0u;
static bal_log_meta_t s_meta;

/* Volcado. */
static uint8_t  s_flush_step = 0u;
static uint32_t s_flush_row = 0u;
static char     s_path[BAL_LOG_PATH_MAX];
static char     s_txt[BAL_LOG_CHUNK + BAL_LOG_ROW_MAX];

/* ========================================================================= */
/* Formato                                                                    */
/* ========================================================================= */

static int16_t q16(float v, float scale)
{
    float x = v * scale;
    x += (x < 0.0f) ? -0.5f : 0.5f;
    if (x > 32767.0f) {
        return 32767;
    }
    if (x < -32767.0f) {
        return -32767;
    }
    return (int16_t)x;
}

static int32_t qf(float v, float scale)
{
    float x = v * scale;
    x += (x < 0.0f) ? -0.5f : 0.5f;
    return (int32_t)x;
}

/* Entero sin signo con al menos min_digits cifras. */
static char *put_u(char *p, uint32_t v, uint8_t min_digits)
{
    char tmp[10];
    uint8_t n = 0u;
    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while ((v != 0u) && (n < sizeof(tmp)));
    while (n < min_digits && n < sizeof(tmp)) {
        tmp[n++] = '0';
    }
    while (n > 0u) {
        *p++ = tmp[--n];
    }
    return p;
}

/* v / 10^dec con dec decimales: put_fixed(1234, 2) -> "12.34". */
static char *put_fixed(char *p, int32_t v, uint8_t dec)
{
    uint32_t div = 1u;
    for (uint8_t k = 0u; k < dec; ++k) {
        div *= 10u;
    }
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    p = put_u(p, (uint32_t)v / div, 1u);
    if (dec > 0u) {
        *p++ = '.';
        p = put_u(p, (uint32_t)v % div, dec);
    }
    return p;
}

static char *put_sep(char *p)
{
    *p++ = ',';
    return p;
}

static uint32_t format_row(char *out, uint32_t row)
{
    const bal_log_rec_t *r = &s_buf[row];
    char *p = out;

    p = put_fixed(p, (int32_t)((row * BAL_LOG_DT_US) / 1000u), 3);   /* t_s */
    p = put_sep(p); p = put_fixed(p, r->ref, 2);
    p = put_sep(p); p = put_fixed(p, r->theta, 2);
    p = put_sep(p); p = put_fixed(p, r->omega, 1);
    p = put_sep(p); p = put_fixed(p, r->omega_ref, 1);
    p = put_sep(p); p = put_fixed(p, r->p, 4);
    p = put_sep(p); p = put_fixed(p, r->i, 4);
    p = put_sep(p); p = put_fixed(p, r->d, 4);
    p = put_sep(p); p = put_fixed(p, r->tau, 4);
    p = put_sep(p); p = put_fixed(p, r->thr, 4);
    p = put_sep(p); p = put_fixed(p, r->vl, 4);
    p = put_sep(p); p = put_fixed(p, r->vr, 4);
    p = put_sep(p); p = put_u(p, r->m1, 1u);
    p = put_sep(p); p = put_u(p, r->m3, 1u);
    p = put_sep(p); p = put_u(p, r->armed, 1u);
    *p++ = '\r';
    *p++ = '\n';
    return (uint32_t)(p - out);
}

/* "# clave,valor" con valor de punto fijo. */
static uint32_t header_fixed(char *out, const char *key, int32_t v, uint8_t dec)
{
    char *p = out;
    const size_t k = strlen(key);
    *p++ = '#';
    *p++ = ' ';
    memcpy(p, key, k);
    p += k;
    *p++ = ',';
    p = put_fixed(p, v, dec);
    *p++ = '\r';
    *p++ = '\n';
    return (uint32_t)(p - out);
}

static uint32_t format_header(char *out)
{
    const bal_log_meta_t *m = &s_meta;
    uint32_t n = 0u;

    n += (uint32_t)sprintf(&out[n], "# balancin_test,%u\r\n# archivo,%s\r\n",
                           (unsigned)m->test, s_path);
    n += header_fixed(&out[n], "ref_deg", qf(m->ref_deg, Q_DEG), 2);
    n += header_fixed(&out[n], "ref_prev_deg", qf(m->ref_prev_deg, Q_DEG), 2);
    n += header_fixed(&out[n], "thr", qf(m->thr, Q_FRAC), 4);
    n += header_fixed(&out[n], "trim_deg", qf(m->trim_deg, Q_DEG), 2);
    n += header_fixed(&out[n], "step_s",
                      (int32_t)((m->step_index * BAL_LOG_DT_US) / 1000u), 3);
    n += header_fixed(&out[n], "rate_hz", (int32_t)BAL_LOG_RATE_HZ, 0);
    n += header_fixed(&out[n], "samples", (int32_t)s_n, 0);
    n += (uint32_t)sprintf(&out[n], "# fin,%c\r\n", m->end);
    n += header_fixed(&out[n], "vbat", (int32_t)m->vbat_cv, 2);
    n += header_fixed(&out[n], "kp", qf(BAL_KP, 100.0f), 2);
    n += header_fixed(&out[n], "ki", qf(BAL_KI, 100.0f), 2);
    n += header_fixed(&out[n], "kd", qf(BAL_KD, 100.0f), 2);
    n += header_fixed(&out[n], "level_gain", qf(BAL_LEVEL_GAIN, 100.0f), 2);
    n += header_fixed(&out[n], "omega_max_dps", qf(BAL_LEVEL_MAX_RATE_DPS, 10.0f), 1);
    n += header_fixed(&out[n], "corr_limit", qf(BAL_CORR_LIMIT, Q_FRAC), 4);
    n += header_fixed(&out[n], "idle", qf(BAL_IDLE, Q_FRAC), 4);
    n += header_fixed(&out[n], "vmax", qf(BAL_MOTOR_LIMIT, Q_FRAC), 4);
    n += (uint32_t)sprintf(&out[n], "t_s,ref_deg,theta_deg,omega_dps,omega_ref_dps,"
                                    "p,i,d,tau,thr,vL,vR,m1_dshot,m3_dshot,armed\r\n");
    return n;
}

/* ========================================================================= */
/* Estado                                                                     */
/* ========================================================================= */

static void end_rec(char reason)
{
    s_meta.end = reason;
    s_flush_step = 0u;
    s_flush_row = 0u;
    s_state = BAL_LOG_FLUSH;
}

static void finish_flush(bool ok)
{
    (void)sd_file_close();

    if (ok) {
        char line[96];
        char *p = line;
        p = put_u(p, s_meta.test, 1u);
        *p++ = ',';
        memcpy(p, s_path, strlen(s_path));
        p += strlen(s_path);
        *p++ = ',';
        p = put_fixed(p, qf(s_meta.ref_deg, Q_DEG), 2);
        *p++ = ',';
        p = put_fixed(p, qf(s_meta.ref_prev_deg, Q_DEG), 2);
        *p++ = ',';
        p = put_fixed(p, qf(s_meta.thr, Q_FRAC), 4);
        *p++ = ',';
        p = put_u(p, s_n, 1u);
        *p++ = ',';
        *p++ = s_meta.end;
        *p++ = '\r';
        *p++ = '\n';
        *p = '\0';
        if (sd_append_line(BAL_LOG_INDEX_FILE, line) != SD_OK) {
            s_errors++;
        }
    } else {
        s_errors++;
    }

    /* El numero avanza aunque falle: no se pisa un archivo a medias. */
    s_next_test = (uint16_t)((s_next_test % 999u) + 1u);
    char num[8];
    char *p = put_u(num, s_next_test, 1u);
    *p = '\0';
    if (sd_write_file(BAL_LOG_NEXT_FILE, num, (uint32_t)strlen(num)) != SD_OK) {
        s_errors++;
    }

    s_n = 0u;
    s_state = BAL_LOG_IDLE;
}

/* ========================================================================= */
/* API                                                                        */
/* ========================================================================= */

void bal_log_init(bool sd_ok)
{
    s_pre_head = 0u;
    s_pre_count = 0u;
    s_n = 0u;
    s_errors = 0u;
    s_next_test = 1u;

    if (!sd_ok || (sd_mkdir(BAL_LOG_DIR) != SD_OK)) {
        s_state = BAL_LOG_OFF;
        return;
    }

    char num[8];
    uint32_t len = 0u;
    if (sd_read_file(BAL_LOG_NEXT_FILE, num, sizeof(num) - 1u, &len) == SD_OK) {
        uint32_t v = 0u;
        for (uint32_t k = 0u; k < len && num[k] >= '0' && num[k] <= '9'; ++k) {
            v = (v * 10u) + (uint32_t)(num[k] - '0');
        }
        if ((v >= 1u) && (v <= 999u)) {
            s_next_test = (uint16_t)v;
        }
    } else {
        /* SD nueva: encabezado del indice. */
        (void)sd_append_line(BAL_LOG_INDEX_FILE,
                             "test,archivo,ref_deg,ref_prev_deg,thr,muestras,fin\r\n");
    }

    s_state = BAL_LOG_IDLE;
}

void bal_log_sample(const bal_log_input_t *in)
{
    if ((in == 0) || (s_state == BAL_LOG_OFF)) {
        return;
    }

    bal_log_rec_t r;
    r.ref = q16(in->ref_deg, Q_DEG);
    r.theta = q16(in->theta_deg, Q_DEG);
    r.omega = q16(in->omega_dps, Q_DPS);
    r.omega_ref = q16(in->omega_ref_dps, Q_DPS);
    r.p = q16(in->p, Q_FRAC);
    r.i = q16(in->i, Q_FRAC);
    r.d = q16(in->d, Q_FRAC);
    r.tau = q16(in->tau, Q_FRAC);
    r.thr = q16(in->thr, Q_FRAC);
    r.vl = q16(in->vl, Q_FRAC);
    r.vr = q16(in->vr, Q_FRAC);
    r.m1 = in->m1_dshot;
    r.m3 = in->m3_dshot;
    r.armed = in->armed ? 1u : 0u;

    s_pre[s_pre_head] = r;
    s_pre_head = (s_pre_head + 1u) % BAL_LOG_PRE_N;
    if (s_pre_count < BAL_LOG_PRE_N) {
        s_pre_count++;
    }

    if (s_state == BAL_LOG_REC) {
        s_buf[s_n++] = r;
        if (s_n >= BAL_LOG_MAX_N) {
            end_rec('C');
        }
    }
}

bal_log_start_t bal_log_start(float ref_deg, float ref_prev_deg,
                              float thr, float trim_deg, uint16_t vbat_cv)
{
    if (s_state == BAL_LOG_OFF) {
        return BAL_LOG_START_NO_SD;
    }
    if (s_state == BAL_LOG_REC) {
        end_rec('R');
        return BAL_LOG_START_BUSY;
    }
    if (s_state == BAL_LOG_FLUSH) {
        return BAL_LOG_START_BUSY;
    }

    /* Pre-trigger en orden cronologico. */
    const uint32_t first = (s_pre_head + BAL_LOG_PRE_N - s_pre_count) % BAL_LOG_PRE_N;
    for (uint32_t k = 0u; k < s_pre_count; ++k) {
        s_buf[k] = s_pre[(first + k) % BAL_LOG_PRE_N];
    }
    s_n = s_pre_count;

    s_meta.test = s_next_test;
    s_meta.ref_deg = ref_deg;
    s_meta.ref_prev_deg = ref_prev_deg;
    s_meta.thr = thr;
    s_meta.trim_deg = trim_deg;
    s_meta.vbat_cv = vbat_cv;
    s_meta.step_index = s_n;
    s_meta.end = 'C';

    /* 8.3: T + numero + P/M + |ref| redondeada (el valor exacto va adentro). */
    int32_t ref_i = qf(ref_deg, 1.0f);
    const char sign = (ref_i < 0) ? 'M' : 'P';
    if (ref_i < 0) {
        ref_i = -ref_i;
    }
    if (ref_i > 99) {
        ref_i = 99;
    }
    char *p = s_path;
    memcpy(p, BAL_LOG_DIR "/T", sizeof(BAL_LOG_DIR "/T") - 1u);
    p += sizeof(BAL_LOG_DIR "/T") - 1u;
    p = put_u(p, s_meta.test, 3u);
    *p++ = sign;
    p = put_u(p, (uint32_t)ref_i, 2u);
    memcpy(p, ".CSV", 5u);

    s_state = BAL_LOG_REC;
    return BAL_LOG_START_OK;
}

void bal_log_stop(void)
{
    if (s_state == BAL_LOG_REC) {
        end_rec('D');
    }
}

void bal_log_task(void)
{
    if (s_state != BAL_LOG_FLUSH) {
        return;
    }

    if (s_flush_step == 0u) {
        if (sd_file_create(s_path) != SD_OK) {
            finish_flush(false);
            return;
        }
        const uint32_t n = format_header(s_txt);
        if (sd_file_append(s_txt, n) != SD_OK) {
            finish_flush(false);
            return;
        }
        s_flush_step = 1u;
        return;
    }

    uint32_t n = 0u;
    while ((s_flush_row < s_n) && (n < BAL_LOG_CHUNK)) {
        n += format_row(&s_txt[n], s_flush_row);
        s_flush_row++;
    }
    if ((n > 0u) && (sd_file_append(s_txt, n) != SD_OK)) {
        finish_flush(false);
        return;
    }
    if (s_flush_row >= s_n) {
        finish_flush(true);
    }
}

bal_log_state_t bal_log_state(void)
{
    return s_state;
}

const char *bal_log_state_name(void)
{
    switch (s_state) {
    case BAL_LOG_IDLE:  return "idle";
    case BAL_LOG_REC:   return "rec";
    case BAL_LOG_FLUSH: return "flush";
    case BAL_LOG_OFF:
    default:            return "off";
    }
}

uint16_t bal_log_test_number(void)
{
    return ((s_state == BAL_LOG_REC) || (s_state == BAL_LOG_FLUSH)) ? s_meta.test
                                                                    : s_next_test;
}

uint32_t bal_log_errors(void)
{
    return s_errors;
}
