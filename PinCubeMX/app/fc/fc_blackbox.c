/**
 ******************************************************************************
 * @file    fc_blackbox.c
 * @brief   Blackbox en la microSD (ver fc_blackbox.h).
 ******************************************************************************
 */

#include "fc_blackbox.h"
#include "fc_rc.h"
#include "fc_state.h"
#include "fc_tasks.h"

#include "arming.h"
#include "failsafe.h"
#include "filter.h"
#include "mixer.h"
#include "pid.h"

#include "driver_sd.h"
#include "sensors_battery.h"
#include "sensors_calib.h"
#include "timebase.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>

#if FC_ENABLE_BLACKBOX

/* Ring en RAM normal (.bss, alineado a 4): el DMA del SDIO lee de aca.
 * 32 KB = ~450 ms a 72 KB/s: cubre las pausas largas de la tarjeta.     */
#define BB_RING_LEN        32768u
#define BB_CHUNK           4096u      /* 8 sectores por escritura */
#define BB_SECTOR          512u
#define BB_MIN_FILE_BYTES  (4u * 1024u * 1024u)
#define BB_WRITE_TIMEOUT_MS 1000u

/* Registro de 72 bytes, little-endian. El orden y las escalas tienen que
 * coincidir con s_fields (la cabecera los lista para el decoder).        */
typedef struct __attribute__((packed)) {
    uint32_t t_us;          /* desde el armado                         */
    int16_t  gyro_raw[3];   /* x10 dps, sin filtrar (signo del PID)     */
    int16_t  gyro[3];       /* x10 dps, filtrado (lo que ve el PID)     */
    int16_t  setpoint[3];   /* x10 dps                                  */
    int16_t  p[3];          /* x10 unidades BF (/1000 = motor)          */
    int16_t  i[3];
    int16_t  d[3];
    int16_t  sum[3];        /* x10 unidades BF, P+I+D recortado         */
    int16_t  angle[2];      /* x10 grados                               */
    int16_t  stick[3];      /* x1000 (-1..+1)                           */
    uint16_t throttle;      /* x1000 (0..1)                             */
    uint16_t motor[4];      /* DShot CH1..CH4 (M4, M3, M2, M1)          */
    uint16_t vbat_cv;       /* centivolts                               */
    uint16_t loop_us;       /* intervalo max entre PIDs desde el ultimo */
    uint8_t  flags;         /* b0 armado b1 failsafe b2 link b3 calib   */
    uint8_t  seq;           /* contador: el decoder detecta el final    */
} bb_record_t;

_Static_assert(sizeof(bb_record_t) == FC_BB_RECORD_LEN, "bb_record_t != 72 B");
_Static_assert((BB_RING_LEN % BB_CHUNK) == 0u, "ring no multiplo del chunk");
_Static_assert((FC_BB_HEADER_LEN % BB_SECTOR) == 0u, "cabecera no multiplo de 512");

static const char s_fields[] =
    "t_us:u32:1 "
    "gyro_raw_roll:i16:0.1 gyro_raw_pitch:i16:0.1 gyro_raw_yaw:i16:0.1 "
    "gyro_roll:i16:0.1 gyro_pitch:i16:0.1 gyro_yaw:i16:0.1 "
    "setpoint_roll:i16:0.1 setpoint_pitch:i16:0.1 setpoint_yaw:i16:0.1 "
    "p_roll:i16:0.1 p_pitch:i16:0.1 p_yaw:i16:0.1 "
    "i_roll:i16:0.1 i_pitch:i16:0.1 i_yaw:i16:0.1 "
    "d_roll:i16:0.1 d_pitch:i16:0.1 d_yaw:i16:0.1 "
    "sum_roll:i16:0.1 sum_pitch:i16:0.1 sum_yaw:i16:0.1 "
    "angle_roll:i16:0.1 angle_pitch:i16:0.1 "
    "stick_roll:i16:0.001 stick_pitch:i16:0.001 stick_yaw:i16:0.001 "
    "throttle:u16:0.001 "
    "motor_ch1:u16:1 motor_ch2:u16:1 motor_ch3:u16:1 motor_ch4:u16:1 "
    "vbat:u16:0.01 loop_us:u16:1 flags:u8:1 seq:u8:1";

static uint32_t s_ring_w[BB_RING_LEN / 4u];
#define s_ring ((uint8_t *)s_ring_w)

static uint32_t s_head = 0u;        /* contadores monotonicos de bytes   */
static uint32_t s_tail = 0u;
static uint32_t s_inflight = 0u;    /* bytes entregados al DMA           */
static uint32_t s_xfer_t0_ms = 0u;

static FIL      s_fil;
static bool     s_file_open = false;
static uint32_t s_lba = 0u;         /* proximo sector a escribir         */
static uint32_t s_file_cap = 0u;    /* bytes reservados                  */
static uint32_t s_written = 0u;     /* bytes ya en la tarjeta            */
static uint16_t s_next_index = 1u;

static uint8_t  s_header[FC_BB_HEADER_LEN];

static bool     s_prev_armed = false;
static uint32_t s_div = 0u;
static uint8_t  s_seq = 0u;
static uint64_t s_t0_us = 0u;
static uint32_t s_last_pid_cyc = 0u;
static uint32_t s_loop_max_us = 0u;

static blackbox_status_t s_st;

/* ------------------------------------------------------------------------- */
/* Ring                                                                       */
/* ------------------------------------------------------------------------- */
static uint32_t bb_used(void)
{
    return s_head - s_tail;
}

static bool bb_push(const void *data, uint32_t len)
{
    if ((BB_RING_LEN - bb_used()) < len) {
        return false;
    }
    const uint8_t *src = (const uint8_t *)data;
    uint32_t pos = s_head % BB_RING_LEN;
    const uint32_t first = (len < (BB_RING_LEN - pos)) ? len : (BB_RING_LEN - pos);
    memcpy(&s_ring[pos], src, first);
    if (first < len) {
        memcpy(&s_ring[0], &src[first], len - first);
    }
    s_head += len;
    return true;
}

static void bb_push_zeros(uint32_t len)
{
    static const uint8_t zeros[BB_SECTOR] = { 0u };
    (void)bb_push(zeros, len);   /* len < 512, siempre entra al vaciar */
}

/* ------------------------------------------------------------------------- */
/* Cabecera                                                                   */
/* ------------------------------------------------------------------------- */
static uint32_t bb_put(uint32_t at, const char *fmt, long a, long b)
{
    if (at >= FC_BB_HEADER_LEN) {
        return at;
    }
    const int n = snprintf((char *)&s_header[at], FC_BB_HEADER_LEN - at, fmt, a, b);
    if (n <= 0) {
        return at;
    }
    const uint32_t end = at + (uint32_t)n;
    return (end < FC_BB_HEADER_LEN) ? end : (FC_BB_HEADER_LEN - 1u);
}

/* nano.specs no imprime floats: valor con 3 decimales a mano. */
static uint32_t bb_put_milli(uint32_t at, const char *key, float v)
{
    if (at >= FC_BB_HEADER_LEN) {
        return at;
    }
    long m = (long)(v * 1000.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
    const char *sign = "";
    if (m < 0) {
        sign = "-";
        m = -m;
    }
    const int n = snprintf((char *)&s_header[at], FC_BB_HEADER_LEN - at,
                           "%s=%s%ld.%03ld\n", key, sign, m / 1000, m % 1000);
    if (n <= 0) {
        return at;
    }
    const uint32_t end = at + (uint32_t)n;
    return (end < FC_BB_HEADER_LEN) ? end : (FC_BB_HEADER_LEN - 1u);
}

static void bb_build_header(void)
{
    memset(s_header, 0, sizeof(s_header));
    uint32_t at = 0u;

    at = bb_put(at, "FCBBL %ld\nrecord_len=%ld\n", (long)FC_BB_FORMAT_VERSION,
                (long)FC_BB_RECORD_LEN);
    at = bb_put(at, "header_len=%ld\nfile=%ld\n", (long)FC_BB_HEADER_LEN,
                (long)s_st.file_index);
    at = bb_put(at, "gyro_hz=%ld\npid_hz=%ld\n", (long)FC_GYRO_RATE_HZ,
                (long)FC_PID_RATE_HZ);
    at = bb_put(at, "log_hz=%ld\npid_enabled=%ld\n",
                (long)(FC_PID_RATE_HZ / FC_BLACKBOX_RATE_DENOM), (long)FC_ENABLE_PID);

    at = bb_put_milli(at, "roll_kp", PID_ROLL_KP);
    at = bb_put_milli(at, "roll_ki", PID_ROLL_KI);
    at = bb_put_milli(at, "roll_kd", PID_ROLL_KD);
    at = bb_put_milli(at, "pitch_kp", PID_PITCH_KP);
    at = bb_put_milli(at, "pitch_ki", PID_PITCH_KI);
    at = bb_put_milli(at, "pitch_kd", PID_PITCH_KD);
    at = bb_put_milli(at, "yaw_kp", PID_YAW_KP);
    at = bb_put_milli(at, "yaw_ki", PID_YAW_KI);
    at = bb_put_milli(at, "yaw_kd", PID_YAW_KD);
    at = bb_put_milli(at, "gain_scale", PID_GAIN_SCALE);
    at = bb_put_milli(at, "d_scale", PID_D_SCALE);
    at = bb_put_milli(at, "sum_limit", PID_SUM_LIMIT);
    at = bb_put_milli(at, "sum_limit_yaw", PID_SUM_LIMIT_YAW);
    at = bb_put_milli(at, "iterm_limit", PID_ITERM_LIMIT);
    at = bb_put_milli(at, "angle_limit_deg", PID_ANGLE_LIMIT_DEG);
    at = bb_put_milli(at, "level_gain", PID_LEVEL_GAIN);
    at = bb_put_milli(at, "gyro_lpf1_hz", FC_GYRO_LPF_HZ);
    at = bb_put_milli(at, "gyro_lpf2_hz", FC_GYRO_LPF2_HZ);
    at = bb_put_milli(at, "dterm_lpf1_hz", PID_DTERM_LPF1_HZ);
    at = bb_put_milli(at, "dterm_lpf2_hz", PID_DTERM_LPF2_HZ);
    at = bb_put_milli(at, "throttle_limit", MIXER_THROTTLE_LIMIT);
    at = bb_put_milli(at, "idle_throttle", MIXER_IDLE_THROTTLE);

    if (at < FC_BB_HEADER_LEN) {
        (void)snprintf((char *)&s_header[at], FC_BB_HEADER_LEN - at,
                       "fields=%s\nend\n", s_fields);
    }
}

/* ------------------------------------------------------------------------- */
/* Archivo                                                                    */
/* ------------------------------------------------------------------------- */
static void bb_fail(bb_error_t err)
{
    s_st.error = err;
    s_st.state = BB_STATE_ERROR;
}

/* Escritura raw esperando el final (solo boot / desarmado). */
static bool bb_raw_write_blocking(const void *buf, uint32_t lba, uint32_t nblocks)
{
    if (sd_raw_write_start(buf, lba, nblocks) != SD_OK) {
        return false;
    }
    const uint32_t t0 = HAL_GetTick();
    for (;;) {
        const sd_raw_state_t r = sd_raw_poll();
        if (r == SD_RAW_DONE) {
            return true;
        }
        if ((r == SD_RAW_ERROR) || ((HAL_GetTick() - t0) > BB_WRITE_TIMEOUT_MS)) {
            return false;
        }
    }
}

/* Reserva el proximo BBnnn.BBL. Bloqueante (desarmado / boot). */
static void bb_prepare(void)
{
    char name[16];
    FILINFO fno;
    uint16_t idx = s_next_index;

    for (; idx <= 999u; ++idx) {
        (void)snprintf(name, sizeof(name), "BB%03u.BBL", (unsigned)idx);
        if (f_stat(name, &fno) == FR_NO_FILE) {
            break;
        }
    }
    if (idx > 999u) {
        s_st.state = BB_STATE_OFF;
        s_st.error = BB_ERR_NOSPACE;
        return;
    }

    uint32_t cap = FC_BLACKBOX_FILE_MB * 1024u * 1024u;
    const uint32_t free_b = sd_free_bytes();
    const uint32_t margin = 1024u * 1024u;
    if (free_b < (BB_MIN_FILE_BYTES + margin)) {
        s_st.state = BB_STATE_OFF;
        s_st.error = BB_ERR_NOSPACE;
        return;
    }
    if (cap > (free_b - margin)) {
        cap = free_b - margin;
    }
    cap -= cap % BB_CHUNK;

    if (sd_contig_create(&s_fil, name, cap, &s_lba) != SD_OK) {
        s_st.state = BB_STATE_OFF;
        s_st.error = BB_ERR_CREATE;
        return;
    }

    /* Los clusters reservados traen lo que habia en la tarjeta (quizas un
     * log borrado). La zona de la cabecera queda en ceros: un archivo que
     * nunca se armo no tiene "FCBBL" y se borra en el proximo boot.      */
    static const uint32_t zero_sector[BB_SECTOR / 4u] = { 0u };
    for (uint32_t k = 0u; k < (FC_BB_HEADER_LEN / BB_SECTOR); ++k) {
        if (!bb_raw_write_blocking(zero_sector, s_lba + k, 1u)) {
            (void)sd_contig_close(&s_fil, 0u);
            (void)f_unlink(name);
            s_st.state = BB_STATE_OFF;
            s_st.error = BB_ERR_WRITE;
            return;
        }
    }

    s_file_open = true;
    s_file_cap = cap;
    s_next_index = (uint16_t)(idx + 1u);
    s_st.file_index = idx;
    s_st.error = BB_ERR_NONE;
    s_st.state = BB_STATE_READY;
    bb_build_header();
}

static void bb_finalize(uint32_t bytes)
{
    if (s_file_open) {
        const sd_status_t st = sd_contig_close(&s_fil, bytes);
        s_file_open = false;
        if (st != SD_OK) {
            bb_fail(BB_ERR_CLOSE);
            return;
        }
    }
    bb_prepare();
}

/* El dron se prende con la bateria: cada boot reserva un archivo aunque
 * no se arme. Si el ultimo BBnnn no tiene cabecera (nunca se grabo), se
 * borra y se reutiliza su numero.                                         */
static void bb_cleanup_unused(void)
{
    char name[16];
    FILINFO fno;
    uint16_t idx = 1u;

    for (; idx <= 999u; ++idx) {
        (void)snprintf(name, sizeof(name), "BB%03u.BBL", (unsigned)idx);
        if (f_stat(name, &fno) == FR_NO_FILE) {
            break;
        }
    }
    s_next_index = idx;
    if (idx <= 1u) {
        return;
    }

    (void)snprintf(name, sizeof(name), "BB%03u.BBL", (unsigned)(idx - 1u));
    uint8_t magic[5] = { 0u };
    UINT n = 0u;
    if (f_open(&s_fil, name, FA_READ) != FR_OK) {
        return;
    }
    const FRESULT fr = f_read(&s_fil, magic, sizeof(magic), &n);
    (void)f_close(&s_fil);
    if ((fr == FR_OK) && ((n < sizeof(magic)) || (memcmp(magic, "FCBBL", 5u) != 0))) {
        if (f_unlink(name) == FR_OK) {
            s_next_index = (uint16_t)(idx - 1u);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */
void blackbox_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.state = BB_STATE_OFF;

    if (sd_mount() != SD_OK) {
        s_st.error = BB_ERR_MOUNT;
        return;
    }
    bb_cleanup_unused();
    bb_prepare();
}

static void bb_start(void)
{
    s_head = 0u;
    s_tail = 0u;
    s_inflight = 0u;
    s_written = 0u;
    s_div = 0u;
    s_seq = 0u;
    s_loop_max_us = 0u;
    s_st.records = 0u;
    s_st.drops = 0u;
    s_st.ring_peak_pct = 0u;
    s_t0_us = timebase_us64();

    (void)bb_push(s_header, FC_BB_HEADER_LEN);
    s_st.bytes = FC_BB_HEADER_LEN;
    s_st.state = BB_STATE_LOGGING;
}

static int16_t bb_i16(float v, float scale)
{
    float s = v * scale;
    if (s > 32767.0f) {
        s = 32767.0f;
    } else if (s < -32768.0f) {
        s = -32768.0f;
    }
    return (int16_t)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}

static uint16_t bb_u16(float v, float scale)
{
    float s = v * scale;
    if (s < 0.0f) {
        s = 0.0f;
    } else if (s > 65535.0f) {
        s = 65535.0f;
    }
    return (uint16_t)(s + 0.5f);
}

void blackbox_record(void)
{
    const bool armed = arming_is_armed();
    const uint32_t now_cyc = timebase_now();

    if (armed && !s_prev_armed && (s_st.state == BB_STATE_READY)) {
        bb_start();
    } else if (!armed && s_prev_armed && (s_st.state == BB_STATE_LOGGING)) {
        s_st.state = BB_STATE_FLUSHING;
    }
    s_prev_armed = armed;

    const uint32_t loop_us = timebase_delta_us(s_last_pid_cyc, now_cyc);
    s_last_pid_cyc = now_cyc;
    if (loop_us > s_loop_max_us) {
        s_loop_max_us = loop_us;
    }

    if (s_st.state != BB_STATE_LOGGING) {
        return;
    }
    if (++s_div < FC_BLACKBOX_RATE_DENOM) {
        return;
    }
    s_div = 0u;

    const fc_state_t *st = fc_state();
    const fc_rc_t *rc = fc_rc_get();
    bb_record_t r;

    r.t_us = (uint32_t)(timebase_us64() - s_t0_us);

    r.gyro_raw[0] = bb_i16(FC_GYRO_ROLL_SIGN * st->imu_si.gx_dps, 10.0f);
    r.gyro_raw[1] = bb_i16(FC_GYRO_PITCH_SIGN * st->imu_si.gy_dps, 10.0f);
    r.gyro_raw[2] = bb_i16(FC_GYRO_YAW_SIGN * st->imu_si.gz_dps, 10.0f);
    for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
        r.gyro[ax] = bb_i16(st->gyro_filt_dps[ax], 10.0f);
        r.setpoint[ax] = bb_i16(pid_setpoint_dps(ax), 10.0f);
        r.p[ax] = bb_i16(pid_pterm(ax), 10.0f);
        r.i[ax] = bb_i16(pid_iterm(ax), 10.0f);
        r.d[ax] = bb_i16(pid_dterm(ax), 10.0f);
        r.sum[ax] = bb_i16(st->pid_out[ax] * PID_MIXER_SCALING, 10.0f);
        r.stick[ax] = bb_i16(rc->stick[ax], 1000.0f);
    }
    r.angle[0] = bb_i16(st->angle_deg[PID_AXIS_ROLL], 10.0f);
    r.angle[1] = bb_i16(st->angle_deg[PID_AXIS_PITCH], 10.0f);
    r.throttle = bb_u16(rc->throttle, 1000.0f);
    for (uint8_t m = 0u; m < 4u; ++m) {
        r.motor[m] = st->motor[m];
    }
    r.vbat_cv = sensors_battery_get()->voltage_cv;
    r.loop_us = (s_loop_max_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s_loop_max_us;
    s_loop_max_us = 0u;

    uint8_t flags = 0u;
    flags |= armed                ? (1u << 0) : 0u;
    flags |= failsafe_active()    ? (1u << 1) : 0u;
    flags |= rc->link_ok          ? (1u << 2) : 0u;
    flags |= sensors_calib_done() ? (1u << 3) : 0u;
    r.flags = flags;
    r.seq = s_seq++;

    if (bb_push(&r, sizeof(r))) {
        s_st.records++;
        s_st.bytes += sizeof(r);
    } else {
        s_st.drops++;
    }

    const uint32_t pct = (bb_used() * 100u) / BB_RING_LEN;
    if (pct > s_st.ring_peak_pct) {
        s_st.ring_peak_pct = (uint8_t)pct;
    }
}

void blackbox_task_write(void)
{
    if ((s_st.state != BB_STATE_LOGGING) && (s_st.state != BB_STATE_FLUSHING)) {
        return;
    }

    if (s_inflight != 0u) {
        const sd_raw_state_t r = sd_raw_poll();
        if (r == SD_RAW_BUSY) {
            if ((HAL_GetTick() - s_xfer_t0_ms) > BB_WRITE_TIMEOUT_MS) {
                bb_fail(BB_ERR_TIMEOUT);
            }
            return;
        }
        if (r == SD_RAW_ERROR) {
            s_inflight = 0u;
            bb_fail(BB_ERR_WRITE);
            return;
        }
        s_tail += s_inflight;
        s_written += s_inflight;
        s_lba += s_inflight / BB_SECTOR;
        s_inflight = 0u;
    }

    uint32_t used = bb_used();
    uint32_t n = 0u;
    if (used >= BB_CHUNK) {
        n = BB_CHUNK;
    } else if ((s_st.state == BB_STATE_FLUSHING) && (used > 0u)) {
        /* Ultimo tramo: se completa el sector con ceros (fuera del tamano
         * logico; el archivo se recorta a s_st.bytes al cerrar).          */
        const uint32_t pad = (BB_SECTOR - (used % BB_SECTOR)) % BB_SECTOR;
        bb_push_zeros(pad);
        used += pad;
        n = used;
    }
    if (n == 0u) {
        return;
    }

    if ((s_written + n) > s_file_cap) {
        s_st.state = BB_STATE_FULL;
        return;
    }
    if (sd_raw_write_start(&s_ring[s_tail % BB_RING_LEN], s_lba, n / BB_SECTOR) != SD_OK) {
        bb_fail(BB_ERR_WRITE);
        return;
    }
    s_inflight = n;
    s_xfer_t0_ms = HAL_GetTick();
}

void blackbox_task_file(void)
{
    if (arming_is_armed()) {
        return;
    }

    switch (s_st.state) {
    case BB_STATE_FLUSHING:
        if ((s_inflight == 0u) && (bb_used() == 0u)) {
            bb_finalize(s_st.bytes);
        }
        break;

    case BB_STATE_FULL:
        /* Lo que quedo en el ring no entra: el archivo termina en lo ya
         * escrito (el decoder descarta un registro cortado al final).     */
        if (s_inflight == 0u) {
            bb_finalize((s_st.bytes < s_written) ? s_st.bytes : s_written);
        }
        break;

    case BB_STATE_ERROR:
        /* Se intenta dejar el archivo con lo escrito; despues queda en
         * ERROR hasta reiniciar (la tarjeta no es confiable).             */
        if (s_file_open && (sd_raw_poll() != SD_RAW_BUSY)) {
            s_inflight = 0u;
            (void)sd_contig_close(&s_fil, (s_st.bytes < s_written) ? s_st.bytes : s_written);
            s_file_open = false;
        }
        break;

    default:
        break;
    }
}

#else  /* !FC_ENABLE_BLACKBOX */

static blackbox_status_t s_st;

void blackbox_init(void) { s_st.state = BB_STATE_OFF; }
void blackbox_record(void) { }
void blackbox_task_write(void) { }
void blackbox_task_file(void) { }

#endif /* FC_ENABLE_BLACKBOX */

const blackbox_status_t *blackbox_status(void)
{
    return &s_st;
}

const char *blackbox_state_name(bb_state_t st)
{
    switch (st) {
    case BB_STATE_OFF:      return "OFF";
    case BB_STATE_READY:    return "READY";
    case BB_STATE_LOGGING:  return "LOGGING";
    case BB_STATE_FLUSHING: return "FLUSHING";
    case BB_STATE_FULL:     return "FULL";
    case BB_STATE_ERROR:    return "ERROR";
    default:                return "?";
    }
}
