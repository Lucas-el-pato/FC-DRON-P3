/**
 ******************************************************************************
 * @file    driver_gps.c
 * @brief   GPS M100 (NMEA) por USART3: RX por IRQ + ring, parser incremental.
 ******************************************************************************
 */

#include "driver_gps.h"
#include "stm32f4xx_hal.h"

#include <string.h>

#if (GPS_RX_RING_SIZE & (GPS_RX_RING_SIZE - 1u)) != 0u
#error "GPS_RX_RING_SIZE debe ser potencia de 2"
#endif

#define GPS_UART            USART3
#define GPS_TERM_MAX        16u   /* el campo mas largo es "dddmm.mmmmm" */
#define GPS_UART_ERR_MASK   (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)

/* ------------------------------------------------------------------------- */
/* Ring (ISR escribe head, gps_process lee tail)                              */
/* ------------------------------------------------------------------------- */
static uint8_t           s_ring[GPS_RX_RING_SIZE];
static volatile uint16_t s_head = 0u;
static volatile uint16_t s_tail = 0u;

static volatile uint32_t s_isr_overruns = 0u;
static volatile uint32_t s_isr_errors = 0u;
static volatile uint32_t s_isr_ring_full = 0u;

/* ------------------------------------------------------------------------- */
/* Parser                                                                     */
/* ------------------------------------------------------------------------- */
typedef enum {
    SENT_OTHER = 0,
    SENT_GGA,
    SENT_RMC
} gps_sentence_t;

typedef struct {
    bool     active;        /* entre '$' y fin de linea          */
    bool     in_checksum;   /* despues del '*'                   */
    uint8_t  parity;
    char     term[GPS_TERM_MAX];
    uint8_t  term_len;
    bool     term_overflow;
    uint8_t  term_num;
    uint8_t  type;          /* gps_sentence_t                    */

    /* Staging de la sentencia en curso (se publica si el checksum da). */
    uint32_t time;     bool time_ok;
    int32_t  lat;      bool lat_ok;
    int32_t  lon;      bool lon_ok;
    uint8_t  quality;  bool quality_ok;
    uint8_t  sats;     bool sats_ok;
    uint16_t hdop;     bool hdop_ok;
    int32_t  alt_cm;   bool alt_ok;
    bool     rmc_active;
    uint16_t speed;    bool speed_ok;
    uint16_t course;   bool course_ok;
} gps_parser_t;

static gps_parser_t s_p;
static gps_data_t   s_data;
static gps_stats_t  s_stats;
static uint8_t      s_good_streak = 0u;   /* GGA buenas seguidas */

/* ------------------------------------------------------------------------- */
/* Helpers de numeros (sin libc/float)                                        */
/* ------------------------------------------------------------------------- */

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* "123.4567" -> entero escalado por 10^decimals (trunca lo que sobra).
 * Acepta signo '-'. false si el campo esta vacio o no es numero.          */
static bool parse_fixed(const char *s, uint8_t decimals, int32_t *out)
{
    bool neg = false;
    bool any = false;
    bool frac = false;
    uint8_t frac_digits = 0u;
    int64_t v = 0;

    if (*s == '-') {
        neg = true;
        s++;
    }
    for (; *s != '\0'; ++s) {
        if (*s == '.') {
            if (frac) {
                return false;
            }
            frac = true;
            continue;
        }
        if (*s < '0' || *s > '9') {
            return false;
        }
        if (frac) {
            if (frac_digits >= decimals) {
                continue;            /* trunca decimales extra */
            }
            frac_digits++;
        }
        v = v * 10 + (*s - '0');
        any = true;
        if (v > 0x7FFFFFFFLL) {
            return false;
        }
    }
    if (!any) {
        return false;
    }
    for (; frac_digits < decimals; ++frac_digits) {
        v *= 10;
        if (v > 0x7FFFFFFFLL) {
            return false;
        }
    }
    *out = (int32_t)(neg ? -v : v);
    return true;
}

/* "ddmm.mmmmm" / "dddmm.mmmmm" -> grados x 1e7 (sin signo).
 * minutos x 1e5 -> grados x 1e7 = min_e5 * 100 / 60 = min_e5 * 5 / 3.    */
static bool parse_coord(const char *s, int32_t *out_e7)
{
    int32_t v;   /* dddmm x 1e5 */
    if (!parse_fixed(s, 5u, &v) || v < 0) {
        return false;
    }
    const int32_t deg = v / 10000000;              /* dd(d) */
    const int32_t min_e5 = v % 10000000;           /* mm.mmmmm x 1e5 */
    if (min_e5 >= 6000000) {
        return false;
    }
    *out_e7 = deg * 10000000 + (min_e5 * 5 + 1) / 3;
    return true;
}

/* ------------------------------------------------------------------------- */
/* Parser NMEA                                                                */
/* ------------------------------------------------------------------------- */

static void parser_start(void)
{
    memset(&s_p, 0, sizeof(s_p));
    s_p.active = true;
}

static void parser_term(void)
{
    const char *t = s_p.term;
    int32_t v;

    if (s_p.term_overflow) {
        return;   /* campo invalido; el resto de la sentencia sigue */
    }

    if (s_p.term_num == 0u) {
        /* "GPGGA", "GNRMC"... el talker no importa. */
        if (s_p.term_len == 5u && t[0] == 'G') {
            if (strcmp(&t[2], "GGA") == 0) {
                s_p.type = SENT_GGA;
            } else if (strcmp(&t[2], "RMC") == 0) {
                s_p.type = SENT_RMC;
            }
        }
        return;
    }

    /* Hora (campo 1 en GGA y RMC). */
    if (s_p.term_num == 1u && s_p.type != SENT_OTHER) {
        if (parse_fixed(t, 0u, &v) && v >= 0) {
            s_p.time = (uint32_t)v;
            s_p.time_ok = true;
        }
        return;
    }

    if (s_p.type == SENT_GGA) {
        switch (s_p.term_num) {
        case 2u: s_p.lat_ok = parse_coord(t, &s_p.lat); break;
        case 3u: if (t[0] == 'S') { s_p.lat = -s_p.lat; } break;
        case 4u: s_p.lon_ok = parse_coord(t, &s_p.lon); break;
        case 5u: if (t[0] == 'W') { s_p.lon = -s_p.lon; } break;
        case 6u:
            if (parse_fixed(t, 0u, &v) && v >= 0 && v <= 9) {
                s_p.quality = (uint8_t)v;
                s_p.quality_ok = true;
            }
            break;
        case 7u:
            if (parse_fixed(t, 0u, &v) && v >= 0 && v <= 99) {
                s_p.sats = (uint8_t)v;
                s_p.sats_ok = true;
            }
            break;
        case 8u:
            if (parse_fixed(t, 2u, &v) && v >= 0) {
                s_p.hdop = (v > 0xFFFF) ? 0xFFFFu : (uint16_t)v;
                s_p.hdop_ok = true;
            }
            break;
        case 9u:
            s_p.alt_ok = parse_fixed(t, 2u, &s_p.alt_cm);
            break;
        default:
            break;
        }
    } else if (s_p.type == SENT_RMC) {
        switch (s_p.term_num) {
        case 2u: s_p.rmc_active = (t[0] == 'A'); break;
        case 3u: s_p.lat_ok = parse_coord(t, &s_p.lat); break;
        case 4u: if (t[0] == 'S') { s_p.lat = -s_p.lat; } break;
        case 5u: s_p.lon_ok = parse_coord(t, &s_p.lon); break;
        case 6u: if (t[0] == 'W') { s_p.lon = -s_p.lon; } break;
        case 7u:
            /* nudos x1000 -> cm/s: 1 kn = 51.4444 cm/s */
            if (parse_fixed(t, 3u, &v) && v >= 0) {
                const uint64_t cms = ((uint64_t)v * 514444u + 5000000u) / 10000000u;
                s_p.speed = (cms > 0xFFFFu) ? 0xFFFFu : (uint16_t)cms;
                s_p.speed_ok = true;
            }
            break;
        case 8u:
            if (parse_fixed(t, 2u, &v) && v >= 0 && v < 36000) {
                s_p.course = (uint16_t)v;
                s_p.course_ok = true;
            }
            break;
        default:
            break;
        }
    }
}

/* Sentencia completa con checksum correcto: publicar. */
static void parser_commit(void)
{
    const uint32_t now = HAL_GetTick();
    gps_data_t *d = &s_data;

    d->last_rx_ms = now;
    if (s_p.time_ok) {
        d->utc_hhmmss = s_p.time;
    }

    if (s_p.type == SENT_GGA) {
        if (s_p.sats_ok)    d->sats = s_p.sats;
        if (s_p.hdop_ok)    d->hdop_x100 = s_p.hdop;
        if (s_p.quality_ok) d->fix_quality = s_p.quality;

        /* Fix con calidad suficiente: si no, la posicion es deriva. */
        const bool good = s_p.quality_ok && (s_p.quality > 0u) &&
                          s_p.lat_ok && s_p.lon_ok &&
                          s_p.sats_ok && (s_p.sats >= GPS_MIN_SATS) &&
                          s_p.hdop_ok && (s_p.hdop <= GPS_MAX_HDOP_X100);
        if (good) {
            if (s_good_streak < 0xFFu) {
                s_good_streak++;
            }
        } else {
            s_good_streak = 0u;
        }

        if (good && s_good_streak >= GPS_GOOD_STREAK) {
            d->lat_e7 = s_p.lat;
            d->lon_e7 = s_p.lon;
            if (s_p.alt_ok) {
                d->alt_cm = s_p.alt_cm;
            }
            d->last_fix_ms = now;
            d->valid = true;
        } else if (!good) {
            d->valid = false;
            d->speed_cms = 0u;
        }
    } else if (s_p.type == SENT_RMC) {
        /* RMC no trae satelites/HDOP: solo actualiza si la ultima GGA fue
         * buena el tiempo suficiente.                                    */
        if (s_p.rmc_active && s_p.lat_ok && s_p.lon_ok &&
            s_good_streak >= GPS_GOOD_STREAK) {
            d->lat_e7 = s_p.lat;
            d->lon_e7 = s_p.lon;
            if (s_p.speed_ok)  d->speed_cms = s_p.speed;
            if (s_p.course_ok) d->course_cdeg = s_p.course;
            d->last_fix_ms = now;
            d->valid = true;
        } else if (!s_p.rmc_active) {
            s_good_streak = 0u;
            d->valid = false;
            d->speed_cms = 0u;
        }
    }
}

static void parser_feed(char c)
{
    if (c == '$') {
        if (s_p.active) {
            s_stats.sentences_bad++;   /* la anterior quedo cortada */
        }
        parser_start();
        return;
    }
    if (!s_p.active) {
        return;
    }

    if (c == '\r' || c == '\n') {
        bool ok = false;
        if (s_p.in_checksum && s_p.term_len == 2u) {
            const int hi = hex_nibble(s_p.term[0]);
            const int lo = hex_nibble(s_p.term[1]);
            ok = (hi >= 0) && (lo >= 0) &&
                 ((uint8_t)((hi << 4) | lo) == s_p.parity);
        }
        if (ok) {
            s_stats.sentences_ok++;
            parser_commit();
        } else {
            s_stats.sentences_bad++;
        }
        s_p.active = false;
        return;
    }

    if (s_p.in_checksum) {
        if (s_p.term_len < 2u) {
            s_p.term[s_p.term_len++] = c;
        } else {
            s_p.term_len = 3u;   /* checksum mas largo -> invalido */
        }
        return;
    }

    if (c == ',' || c == '*') {
        s_p.term[s_p.term_len] = '\0';
        parser_term();
        s_p.term_num++;
        s_p.term_len = 0u;
        s_p.term_overflow = false;
        if (c == '*') {
            s_p.in_checksum = true;
        } else {
            s_p.parity ^= (uint8_t)c;
        }
        return;
    }

    s_p.parity ^= (uint8_t)c;
    if (s_p.term_len < (GPS_TERM_MAX - 1u)) {
        s_p.term[s_p.term_len++] = c;
    } else {
        s_p.term_overflow = true;
    }
}

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */

void gps_init(void)
{
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    GPS_UART->CR1 &= ~USART_CR1_RXNEIE;

    s_head = 0u;
    s_tail = 0u;
    s_isr_overruns = 0u;
    s_isr_errors = 0u;
    s_isr_ring_full = 0u;
    memset(&s_p, 0, sizeof(s_p));
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_stats, 0, sizeof(s_stats));
    s_good_streak = 0u;

    /* El GPS ya transmite cuando el MCU arranca: un ORE pendiente no deja
     * arrancar la RX (gotcha de Comms_CCTE). SR y luego DR lo limpia.    */
    (void)GPS_UART->SR;
    (void)GPS_UART->DR;

    HAL_NVIC_SetPriority(USART3_IRQn, GPS_NVIC_PRIO, 0u);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
    GPS_UART->CR1 |= USART_CR1_RXNEIE;   /* RXNE y ORE disparan el IRQ */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void gps_uart_irq(void)
{
    const uint32_t sr = GPS_UART->SR;
    if ((sr & (USART_SR_RXNE | GPS_UART_ERR_MASK)) == 0u) {
        return;
    }

    /* Leer DR despues de SR limpia RXNE y los flags de error. */
    const uint8_t b = (uint8_t)GPS_UART->DR;

    if ((sr & USART_SR_ORE) != 0u) {
        s_isr_overruns++;
    }
    if ((sr & (USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0u) {
        s_isr_errors++;
    }
    if ((sr & USART_SR_RXNE) == 0u) {
        return;
    }

    const uint16_t head = s_head;
    const uint16_t next = (uint16_t)((head + 1u) & (GPS_RX_RING_SIZE - 1u));
    if (next == s_tail) {
        s_isr_ring_full++;
        return;
    }
    s_ring[head] = b;
    s_head = next;
}

void USART3_IRQHandler(void)
{
    gps_uart_irq();
}

void gps_process(void)
{
    uint16_t tail = s_tail;
    const uint16_t head = s_head;

    while (tail != head) {
        parser_feed((char)s_ring[tail]);
        tail = (uint16_t)((tail + 1u) & (GPS_RX_RING_SIZE - 1u));
        s_stats.rx_bytes++;
    }
    s_tail = tail;

    if (s_data.valid &&
        (HAL_GetTick() - s_data.last_fix_ms) > GPS_FIX_TIMEOUT_MS) {
        s_good_streak = 0u;
        s_data.valid = false;
        s_data.speed_cms = 0u;
    }
}

const gps_data_t *gps_get(void)
{
    return &s_data;
}

void gps_stats_get(gps_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_stats;
    out->uart_overruns = s_isr_overruns;
    out->uart_errors = s_isr_errors;
    out->ring_overflows = s_isr_ring_full;
}
