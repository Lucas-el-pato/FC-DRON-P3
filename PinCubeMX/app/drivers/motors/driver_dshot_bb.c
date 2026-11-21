/**
 ******************************************************************************
 * @file    driver_dshot_bb.c
 * @brief   DShot300 bidireccional por bitbang (ver driver_dshot_bb.h).
 *
 *          Los streams se programan por registro (es mas rapido que HAL y
 *          hay que cambiar de direccion en cada ciclo). Las IRQ de CubeMX
 *          (DMA2_Stream2/4_IRQHandler -> HAL_DMA_IRQHandler) limpian los
 *          flags y llaman a bb_dma_cplt / bb_dma_error, registrados con
 *          HAL_DMA_RegisterCallback.
 ******************************************************************************
 */

/* Este archivo se compila optimizado aunque el proyecto este en -O0: la
 * decodificacion corre en cada PID (2 kHz) y en -O0 costaba ~270 us por
 * ciclo (blackbox BB010). Solo afecta a las funciones de este archivo.  */
#pragma GCC optimize ("O2")

#include "driver_dshot_bb.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include <string.h>

extern DMA_HandleTypeDef hdma_tim1_ch2;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

#define BB_PORTS      2u
#define BB_PORT_A     0u     /* GPIOA: M4, M2, M1 -> DMA2 Stream2 / TIM1_CH2 */
#define BB_PORT_B     1u     /* GPIOB: M3         -> DMA2 Stream4 / TIM1_CH4 */

/* Canal 6, prioridad muy alta, incrementa memoria, IRQ de fin y de error. */
#define BB_CR_COMMON  (DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2 | DMA_SxCR_PL | \
                       DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE)
/* Envio: memoria -> GPIOx->BSRR, palabras de 32 bits. */
#define BB_CR_TX      (BB_CR_COMMON | DMA_SxCR_DIR_0 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1)
/* Lectura: GPIOx->IDR -> memoria, medias palabras. */
#define BB_CR_RX      (BB_CR_COMMON | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0)

/* Ciclos dshot_bb_write seguidos con el ciclo anterior sin terminar antes
 * de cortarlo (normalmente termina en ~180 us, un PID dura 500 us).      */
#define BB_STUCK_CALLS 3u

typedef enum {
    BB_IDLE = 0,
    BB_TX,
    BB_RX
} bb_phase_t;

typedef enum {
    BB_DEC_OK = 0,
    BB_DEC_NO_REPLY,
    BB_DEC_BAD
} bb_decode_t;

typedef struct {
    GPIO_TypeDef      *gpio;
    DMA_HandleTypeDef *hdma;
    volatile uint32_t *ifcr;        /* LIFCR / HIFCR del stream           */
    uint32_t           ifcr_mask;   /* todos los flags del stream         */
    uint16_t           pins;        /* pines de motor del puerto          */
    uint32_t           moder_mask;  /* 2 bits por pin                     */
    uint32_t           moder_out;   /* 01 = salida, por pin               */
} bb_port_t;

typedef struct {
    uint8_t  port;
    uint16_t pin;
} bb_motor_t;

/* Indice del driver (canal TIM2 CH1..CH4) -> puerto y pin. */
static const bb_motor_t s_motor[MOTORS_COUNT] = {
    { BB_PORT_A, M4_Pin },   /* CH1 = M4 = PA15 */
    { BB_PORT_B, M3_Pin },   /* CH2 = M3 = PB3  */
    { BB_PORT_A, M2_Pin },   /* CH3 = M2 = PA2  */
    { BB_PORT_A, M1_Pin },   /* CH4 = M1 = PA3  */
};

/* GCR de 5 bits -> nibble (0xFF = codigo invalido). */
static const uint8_t s_gcr[32] = {
    0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
    0xFFu, 0x09u, 0x0Au, 0x0Bu, 0xFFu, 0x0Du, 0x0Eu, 0x0Fu,
    0xFFu, 0xFFu, 0x02u, 0x03u, 0xFFu, 0x05u, 0x06u, 0x07u,
    0xFFu, 0x00u, 0x08u, 0x01u, 0xFFu, 0x04u, 0x0Cu, 0xFFu,
};

static bb_port_t s_port[BB_PORTS];
static bool      s_ports_built = false;

/* Buffers en RAM normal (.bss): la CCM del F405 no la ve el DMA. */
static uint32_t s_tx[BB_PORTS][DSHOT_BB_TX_LEN];
static uint16_t s_rx[BB_PORTS][DSHOT_BB_RX_SAMPLES];

static volatile uint8_t  s_phase = BB_IDLE;
static volatile uint8_t  s_pending = 0u;      /* streams sin terminar     */
static volatile bool     s_rx_ready = false;  /* hay lectura sin decodificar */
static volatile uint32_t s_dma_errors = 0u;
static uint8_t           s_busy_calls = 0u;
static bool              s_inited = false;
static volatile bool     s_hold = true;       /* lineas en bajo, sin frames */
static motors_bidir_stats_t s_stats;

/* ------------------------------------------------------------------------- */
/* Puertos y pines                                                            */
/* ------------------------------------------------------------------------- */

static void bb_ports_build(void)
{
    if (s_ports_built) {
        return;
    }

    s_port[BB_PORT_A].gpio = GPIOA;
    s_port[BB_PORT_A].hdma = &hdma_tim1_ch2;
    s_port[BB_PORT_A].ifcr = &DMA2->LIFCR;
    s_port[BB_PORT_A].ifcr_mask = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 |
                                  DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;

    s_port[BB_PORT_B].gpio = GPIOB;
    s_port[BB_PORT_B].hdma = &hdma_tim1_ch4_trig_com;
    s_port[BB_PORT_B].ifcr = &DMA2->HIFCR;
    s_port[BB_PORT_B].ifcr_mask = DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4 |
                                  DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4;

    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        s_port[p].pins = 0u;
        s_port[p].moder_mask = 0u;
        s_port[p].moder_out = 0u;
    }
    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        s_port[s_motor[ch].port].pins |= s_motor[ch].pin;
    }
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        for (uint32_t n = 0u; n < 16u; ++n) {
            if ((s_port[p].pins & (1u << n)) != 0u) {
                s_port[p].moder_mask |= 3u << (2u * n);
                s_port[p].moder_out  |= 1u << (2u * n);
            }
        }
    }
    s_ports_built = true;
}

/* Salida en alto (BSRR antes que MODER: el pin pasa directo a alto). */
static void bb_pins_output_high(void)
{
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        GPIO_TypeDef *g = s_port[p].gpio;
        g->BSRR = s_port[p].pins;
        g->MODER = (g->MODER & ~s_port[p].moder_mask) | s_port[p].moder_out;
    }
}

/* Salida en bajo (retenido: el ESC tiene que arrancar viendo la linea baja). */
static void bb_pins_output_low(void)
{
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        GPIO_TypeDef *g = s_port[p].gpio;
        g->BSRR = (uint32_t)s_port[p].pins << 16;
        g->MODER = (g->MODER & ~s_port[p].moder_mask) | s_port[p].moder_out;
    }
}

/* Entrada con pull-up (el pull-up ya quedo configurado en el init). */
static void bb_pins_input(void)
{
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        s_port[p].gpio->MODER &= ~s_port[p].moder_mask;
    }
}

/* Push-pull, velocidad alta, pull-up (para la lectura) y salida en bajo. */
static void bb_pins_setup(void)
{
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        GPIO_TypeDef *g = s_port[p].gpio;
        const uint32_t two_bits = s_port[p].moder_mask;    /* 11 por pin */
        const uint32_t pull_up = s_port[p].moder_out;      /* 01 por pin */
        const uint32_t speed_high = s_port[p].moder_out << 1u;   /* 10 por pin */

        g->BSRR = (uint32_t)s_port[p].pins << 16;
        g->OTYPER &= ~(uint32_t)s_port[p].pins;            /* push-pull  */
        g->OSPEEDR = (g->OSPEEDR & ~two_bits) | speed_high;
        g->PUPDR = (g->PUPDR & ~two_bits) | pull_up;
    }
    bb_pins_output_low();
}

/* ------------------------------------------------------------------------- */
/* TIM1 y streams                                                             */
/* ------------------------------------------------------------------------- */

static void bb_timer_stop(void)
{
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->DIER &= ~(TIM_DIER_CC2DE | TIM_DIER_CC4DE);
}

static void bb_timer_start(void)
{
    TIM1->CNT = 0u;
    TIM1->SR = 0u;
    TIM1->DIER |= TIM_DIER_CC2DE | TIM_DIER_CC4DE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void bb_stream_disable(const bb_port_t *p)
{
    DMA_Stream_TypeDef *s = p->hdma->Instance;
    s->CR &= ~DMA_SxCR_EN;
    for (uint32_t i = 0u; ((s->CR & DMA_SxCR_EN) != 0u) && (i < 1000u); ++i) {
    }
    *p->ifcr = p->ifcr_mask;
}

static void bb_stream_start(const bb_port_t *p, uint32_t cr, uint32_t periph,
                            uint32_t mem, uint32_t count)
{
    DMA_Stream_TypeDef *s = p->hdma->Instance;

    bb_stream_disable(p);
    p->hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    s->PAR = periph;
    s->M0AR = mem;
    s->NDTR = count;
    s->CR = cr;
    /* s_tx se escribe sin volatile (y este archivo va en -O2): la barrera
     * deja el frame en RAM antes de que el DMA empiece a leerlo.          */
    __DMB();
    s->CR = cr | DMA_SxCR_EN;
}

/* Corta todo (timer, streams) y deja los pines en salida: en alto (reposo)
 * o en bajo si esta retenido.                                            */
static void bb_stop_all(void)
{
    bb_timer_stop();
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        bb_stream_disable(&s_port[p]);
    }
    if (s_hold) {
        bb_pins_output_low();
    } else {
        bb_pins_output_high();
    }
    s_pending = 0u;
    s_phase = BB_IDLE;
}

/* ------------------------------------------------------------------------- */
/* IRQ (contexto DMA2_Stream2/4_IRQHandler, prioridad 0)                      */
/* ------------------------------------------------------------------------- */

static void bb_start_rx(void)
{
    bb_timer_stop();
    bb_pins_input();

    s_pending = BB_PORTS;
    s_phase = BB_RX;
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        bb_stream_start(&s_port[p], BB_CR_RX, (uint32_t)&s_port[p].gpio->IDR,
                        (uint32_t)s_rx[p], DSHOT_BB_RX_SAMPLES);
    }
    bb_timer_start();
}

static void bb_finish_rx(void)
{
    bb_timer_stop();
    bb_pins_output_high();
    s_phase = BB_IDLE;
    s_rx_ready = true;
}

/* Un callback por stream; el que termina segundo cambia de fase. */
static void bb_dma_cplt(DMA_HandleTypeDef *hdma)
{
    (void)hdma;

    if (s_pending > 0u) {
        s_pending--;
    }
    if (s_pending != 0u) {
        return;
    }

    if (s_phase == BB_TX) {
        bb_start_rx();
    } else if (s_phase == BB_RX) {
        bb_finish_rx();
    }
}

static void bb_dma_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    s_dma_errors++;
    s_rx_ready = false;
    bb_stop_all();
}

/* ------------------------------------------------------------------------- */
/* Decodificacion de la respuesta (contexto tarea)                            */
/* ------------------------------------------------------------------------- */

/* Respuesta de un motor: 21 bits, cada cambio de nivel = 1, el primero es
 * el bit de inicio (bajo despues del reposo en alto). El ultimo tramo en
 * alto se confunde con el reposo: su largo se deduce de los 21 bits.
 *
 * Se recorre cada puerto UNA vez para todos sus motores y solo se trabaja
 * en las muestras donde cambia algun pin (XOR con la anterior): ~21
 * cambios por motor en ~180 muestras.                                    */

#define BB_DEC_MIN_ROOM  (18u * DSHOT_BB_RX_PER_BIT)   /* muestras minimas */

typedef struct {
    uint32_t value;       /* bits acumulados, el primero queda en el 20   */
    uint32_t bits;
    uint32_t run_start;   /* muestra donde empezo el tramo actual         */
    uint32_t erpm;
    uint16_t pin;
    uint8_t  ch;
    uint8_t  result;      /* bb_decode_t                                  */
    bool     started;     /* ya se vio el bit de inicio                   */
    bool     high;        /* nivel del tramo actual                       */
    bool     done;
} bb_dec_state_t;

/* 21 bits -> eRPM: GCR, CRC invertido y periodo electrico. */
static bb_decode_t bb_decode_value(uint32_t value, uint32_t *erpm)
{
    /* Sin el bit de inicio quedan 20 bits = 4 codigos GCR de 5 bits. */
    const uint32_t gcr = value & 0xFFFFFu;
    uint32_t word = 0u;
    for (uint32_t q = 0u; q < 4u; ++q) {
        const uint8_t nib = s_gcr[(gcr >> (5u * q)) & 0x1Fu];
        if (nib == 0xFFu) {
            return BB_DEC_BAD;
        }
        word |= (uint32_t)nib << (4u * q);
    }

    /* CRC invertido: el XOR de los 4 nibbles da 0xF. */
    uint32_t crc = word ^ (word >> 8);
    crc ^= crc >> 4;
    if ((crc & 0x0Fu) != 0x0Fu) {
        return BB_DEC_BAD;
    }

    const uint32_t data = word >> 4;                 /* eee mmmmmmmmm */
    if (data == 0x0FFFu) {
        *erpm = 0u;                                  /* motor parado  */
        return BB_DEC_OK;
    }
    const uint32_t period_us = (data & 0x1FFu) << (data >> 9);
    if (period_us == 0u) {
        return BB_DEC_BAD;
    }
    *erpm = (60000000u + (period_us / 2u)) / period_us;
    return BB_DEC_OK;
}

/* Cambio de nivel del pin del motor en la muestra i. true = termino. */
static bool bb_decode_edge(bb_dec_state_t *d, uint32_t i)
{
    if (!d->started) {
        /* Primer bajo despues del reposo: bit de inicio. */
        if ((i + BB_DEC_MIN_ROOM) > DSHOT_BB_RX_SAMPLES) {
            d->result = BB_DEC_NO_REPLY;
            return true;
        }
        d->started = true;
        d->run_start = i;
        d->high = false;
        return false;
    }

    uint32_t len = (i - d->run_start + (DSHOT_BB_RX_PER_BIT / 2u)) / DSHOT_BB_RX_PER_BIT;
    if (len == 0u) {
        len = 1u;
    }
    d->bits += len;
    if (d->bits > 21u) {
        d->result = BB_DEC_BAD;
        return true;
    }
    d->value = (d->value << len) | (1u << (len - 1u));
    d->run_start = i;
    d->high = !d->high;
    if (d->bits == 21u) {
        d->result = (uint8_t)bb_decode_value(d->value, &d->erpm);
        return true;
    }
    return false;
}

/* Fin de la ventana sin completar los 21 bits. */
static void bb_decode_window_end(bb_dec_state_t *d)
{
    if (!d->started) {
        d->result = BB_DEC_NO_REPLY;
        return;
    }
    /* Termina en alto (reposo): falta el ultimo tramo. Si termina en bajo
     * la ventana corto la respuesta.                                      */
    if (!d->high || (d->bits < 18u)) {
        d->result = BB_DEC_BAD;
        return;
    }
    const uint32_t rest = 21u - d->bits;
    d->value = (d->value << rest) | (1u << (rest - 1u));
    d->result = (uint8_t)bb_decode_value(d->value, &d->erpm);
}

/* Una pasada por las muestras de un puerto para todos sus motores. */
static void bb_decode_port(uint8_t port, bb_dec_state_t *st, uint8_t count)
{
    const uint16_t *buf = s_rx[port];
    uint16_t prev = buf[DSHOT_BB_RX_SKIP];
    uint16_t active = 0u;

    for (uint8_t k = 0u; k < count; ++k) {
        bb_dec_state_t *d = &st[k];
        d->value = 0u;
        d->bits = 0u;
        d->erpm = 0u;
        d->done = false;
        d->started = false;
        d->high = true;
        d->result = BB_DEC_NO_REPLY;
        if ((prev & d->pin) == 0u) {
            /* Ya en bajo en la primera muestra util: ahi empieza. */
            d->started = true;
            d->run_start = DSHOT_BB_RX_SKIP;
            d->high = false;
        }
        active |= d->pin;
    }

    for (uint32_t i = DSHOT_BB_RX_SKIP + 1u; (i < DSHOT_BB_RX_SAMPLES) && (active != 0u); ++i) {
        const uint16_t cur = buf[i];
        const uint16_t edges = (uint16_t)((cur ^ prev) & active);
        prev = cur;
        if (edges == 0u) {
            continue;
        }
        for (uint8_t k = 0u; k < count; ++k) {
            bb_dec_state_t *d = &st[k];
            if (((edges & d->pin) != 0u) && bb_decode_edge(d, i)) {
                d->done = true;
                active &= (uint16_t)~d->pin;
            }
        }
    }

    for (uint8_t k = 0u; k < count; ++k) {
        if (!st[k].done) {
            bb_decode_window_end(&st[k]);
        }
    }
}

static void bb_decode_all(void)
{
    bb_dec_state_t st[BB_PORTS][MOTORS_COUNT];
    uint8_t count[BB_PORTS] = { 0u, 0u };
    uint8_t ok_mask = 0u;

    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        const uint8_t p = s_motor[ch].port;
        st[p][count[p]].pin = s_motor[ch].pin;
        st[p][count[p]].ch = ch;
        count[p]++;
    }

    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        bb_decode_port(p, st[p], count[p]);
        for (uint8_t k = 0u; k < count[p]; ++k) {
            const bb_dec_state_t *d = &st[p][k];
            if (d->result == BB_DEC_OK) {
                s_stats.ok[d->ch]++;
                s_stats.erpm[d->ch] = d->erpm;
                ok_mask |= (uint8_t)(1u << d->ch);
            } else if (d->result == BB_DEC_NO_REPLY) {
                s_stats.no_reply[d->ch]++;
            } else {
                s_stats.bad[d->ch]++;
            }
        }
    }
    s_stats.last_ok_mask = ok_mask;
    s_stats.frames++;
}

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */

mot_status_t dshot_bb_init(void)
{
    if ((M4_GPIO_Port != GPIOA) || (M2_GPIO_Port != GPIOA) ||
        (M1_GPIO_Port != GPIOA) || (M3_GPIO_Port != GPIOB)) {
        return MOT_ERR_INIT;   /* el mapa de puertos de arriba no aplica */
    }
    /* MX_TIM1_Init() tiene que haber corrido (crea y asocia los streams). */
    if ((hdma_tim1_ch2.Instance != DMA2_Stream2) ||
        (hdma_tim1_ch4_trig_com.Instance != DMA2_Stream4)) {
        return MOT_ERR_INIT;
    }

    bb_ports_build();
    s_hold = true;
    dshot_bb_abort();

    if ((HAL_DMA_RegisterCallback(&hdma_tim1_ch2, HAL_DMA_XFER_CPLT_CB_ID, bb_dma_cplt) != HAL_OK) ||
        (HAL_DMA_RegisterCallback(&hdma_tim1_ch2, HAL_DMA_XFER_ERROR_CB_ID, bb_dma_error) != HAL_OK) ||
        (HAL_DMA_RegisterCallback(&hdma_tim1_ch4_trig_com, HAL_DMA_XFER_CPLT_CB_ID, bb_dma_cplt) != HAL_OK) ||
        (HAL_DMA_RegisterCallback(&hdma_tim1_ch4_trig_com, HAL_DMA_XFER_ERROR_CB_ID, bb_dma_error) != HAL_OK)) {
        return MOT_ERR_DMA;
    }

    /* Reloj de TIM1: PCLK2, x2 si APB2 tiene divisor (RM0090 6.2). */
    uint32_t tclk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
        tclk *= 2u;
    }
    const uint32_t ticks = tclk / DSHOT_BB_STEP_HZ;   /* 168 MHz -> 112 */
    if (ticks < 16u) {
        return MOT_ERR_INIT;
    }

    bb_timer_stop();
    TIM1->PSC = 0u;
    TIM1->ARR = ticks - 1u;
    TIM1->CCR2 = 1u;                /* pedido de DMA 1 tick despues de 0 */
    TIM1->CCR4 = 1u;
    TIM1->CR2 &= ~TIM_CR2_CCDS;     /* DMA en el evento de compare       */
    TIM1->EGR = TIM_EGR_UG;         /* carga PSC/ARR                     */
    TIM1->SR = 0u;

    /* Pasos fijos de cada bit: 0 baja todo, 4 sube todo. El 2 (sube los
     * motores con bit 0) se escribe en cada frame.                       */
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        for (uint32_t b = 0u; b < 16u; ++b) {
            uint32_t *slot = &s_tx[p][b * DSHOT_BB_STEPS_PER_BIT];
            slot[0] = (uint32_t)s_port[p].pins << 16;
            slot[1] = 0u;
            slot[2] = 0u;
            slot[3] = 0u;
            slot[4] = s_port[p].pins;
        }
    }

    bb_pins_setup();   /* en bajo: retenido hasta dshot_bb_hold(false) */

    memset(&s_stats, 0, sizeof(s_stats));
    s_dma_errors = 0u;
    s_busy_calls = 0u;
    s_rx_ready = false;
    s_phase = BB_IDLE;
    s_inited = true;
    return MOT_OK;
}

mot_status_t dshot_bb_write(const uint16_t value[MOTORS_COUNT], uint8_t telem_mask)
{
    if (!s_inited || s_hold) {
        return MOT_ERR_INIT;
    }
    if (s_phase != BB_IDLE) {
        if (++s_busy_calls >= BB_STUCK_CALLS) {
            dshot_bb_abort();
            s_stats.stuck++;
        }
        return MOT_ERR_DMA;
    }
    s_busy_calls = 0u;

    /* Normalmente ya la decodifico dshot_bb_service(); si no llego a
     * tiempo se decodifica aca (antes de reusar el buffer de lectura).    */
    dshot_bb_service();

    /* Frame invertido: CRC negado (spec DShot bidireccional). */
    uint16_t frame[MOTORS_COUNT];
    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        uint16_t v = value[ch];
        if (v > 2047u) {
            v = 2047u;
        }
        const uint16_t packet = (uint16_t)((v << 1) | ((telem_mask >> ch) & 0x1u));
        const uint16_t crc = (uint16_t)(~(packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0Fu);
        frame[ch] = (uint16_t)((packet << 4) | crc);
    }

    for (uint32_t b = 0u; b < 16u; ++b) {
        const uint32_t shift = 15u - b;
        uint32_t zero_a = 0u;
        uint32_t zero_b = 0u;
        for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
            if (((frame[ch] >> shift) & 0x1u) == 0u) {
                if (s_motor[ch].port == BB_PORT_A) {
                    zero_a |= s_motor[ch].pin;
                } else {
                    zero_b |= s_motor[ch].pin;
                }
            }
        }
        s_tx[BB_PORT_A][(b * DSHOT_BB_STEPS_PER_BIT) + 2u] = zero_a;
        s_tx[BB_PORT_B][(b * DSHOT_BB_STEPS_PER_BIT) + 2u] = zero_b;
    }

    bb_timer_stop();
    bb_pins_output_high();
    s_pending = BB_PORTS;
    s_phase = BB_TX;
    for (uint8_t p = 0u; p < BB_PORTS; ++p) {
        bb_stream_start(&s_port[p], BB_CR_TX, (uint32_t)&s_port[p].gpio->BSRR,
                        (uint32_t)s_tx[p], DSHOT_BB_TX_LEN);
    }
    bb_timer_start();
    return MOT_OK;
}

void dshot_bb_service(void)
{
    if (!s_rx_ready) {
        return;
    }
    s_rx_ready = false;
    __DMB();   /* el DMA ya escribio s_rx antes de la IRQ de fin */
    bb_decode_all();
}

bool dshot_bb_busy(void)
{
    return s_phase != BB_IDLE;
}

void dshot_bb_abort(void)
{
    bb_ports_build();
    if ((hdma_tim1_ch2.Instance == NULL) || (hdma_tim1_ch4_trig_com.Instance == NULL)) {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bb_stop_all();
    s_rx_ready = false;
    __set_PRIMASK(primask);
}

void dshot_bb_hold(bool hold)
{
    if (!s_inited) {
        return;
    }
    if (hold) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_hold = true;
        bb_stop_all();          /* deja las lineas en bajo */
        s_rx_ready = false;
        __set_PRIMASK(primask);
    } else if (s_hold) {
        s_hold = false;
        s_busy_calls = 0u;
        bb_pins_output_high();  /* reposo; el proximo write manda frame */
    }
}

bool dshot_bb_held(void)
{
    return s_hold;
}

const motors_bidir_stats_t *dshot_bb_stats(void)
{
    s_stats.dma_errors = s_dma_errors;
    return &s_stats;
}
