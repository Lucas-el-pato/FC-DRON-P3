/**
 ******************************************************************************
 * @file    test_imu_diag.c
 * @brief   Test de diagnostico del bus SPI del IMU LSM6DSV16X.
 *
 *          Pensado para destrabar el caso "WHO_AM_I no llega": no asume
 *          ningun mapeo de pines y verifica empiricamente el cableado real.
 *
 *          Hace 3 fases consecutivas:
 *
 *          PHASE A - Caracterizacion electrica de pines candidatos.
 *            Para cada pin lo configura como input con pull-up y luego
 *            con pull-down, e imprime ambas lecturas. La combinacion
 *            permite inferir si la linea esta flotante, conectada a GND
 *            fuerte, conectada a VCC fuerte, o controlada por un driver.
 *
 *          PHASE A2 - Cross-talk / shorts entre pines.
 *            Setea cada pin como output HIGH y luego LOW, leyendo los
 *            otros tres como input. Si algun input "sigue" al output,
 *            hay un short fisico o un loop accidental.
 *
 *          PHASE B - Brute-force del mapeo SPI.
 *            Itera las 24 permutaciones posibles de {SCK, MOSI, MISO, CS}
 *            sobre los 4 pines candidatos {PA5, PA6, PA7, PC5} y para
 *            cada una intenta leer WHO_AM_I con bit-bang. Si alguna
 *            devuelve 0x70 reporta el mapeo y se queda con LED verde.
 *
 *          Salida: por USB CDC (consola.c) + LEDs (verde si encuentra
 *          el mapeo, rojo si las 24 fallan).
 ******************************************************************************
 */

#include "console.h"
#include "driver_imu.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Lista de pines candidatos. El orden importa: indices 0..3 se usan en      */
/* la tabla de permutaciones.                                                 */
/* ------------------------------------------------------------------------- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} pin_def_t;

static const pin_def_t kPins[4] = {
    { GPIOA, GPIO_PIN_5, "PA5" },
    { GPIOA, GPIO_PIN_6, "PA6" },
    { GPIOA, GPIO_PIN_7, "PA7" },
    { GPIOC, GPIO_PIN_5, "PC5" },
};

/* ------------------------------------------------------------------------- */
/* Helpers de configuracion rapida de un pin.                                 */
/* ------------------------------------------------------------------------- */
static void pin_as_input(const pin_def_t *p, uint32_t pull)
{
    GPIO_InitTypeDef gi = { 0 };
    gi.Pin   = p->pin;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = pull;  /* GPIO_NOPULL | GPIO_PULLUP | GPIO_PULLDOWN */
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(p->port, &gi);
}

static void pin_as_output(const pin_def_t *p)
{
    GPIO_InitTypeDef gi = { 0 };
    gi.Pin   = p->pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(p->port, &gi);
}

static inline void pin_write(const pin_def_t *p, uint8_t level)
{
    if (level) {
        p->port->BSRR = p->pin;
    } else {
        p->port->BSRR = ((uint32_t)p->pin) << 16;
    }
}

static inline uint8_t pin_read(const pin_def_t *p)
{
    return (p->port->IDR & p->pin) ? 1u : 0u;
}

/* Pequeno delay para que el pull-up/pull-down interno asiente la linea
 * antes de leer. Es generoso (cientos de us) porque no es critico aca.   */
static void settle_delay(void)
{
    for (volatile uint32_t i = 0u; i < 5000u; ++i) {
        __NOP();
    }
}

/* Delay corto para bit-bang del WHO_AM_I (~1 MHz SCK). */
static inline void bb_delay(void)
{
    for (volatile uint32_t i = 0u; i < 8u; ++i) {
        __NOP();
    }
}

/* ------------------------------------------------------------------------- */
/* PHASE A: caracterizacion pull-up vs pull-down de cada pin.                 */
/* ------------------------------------------------------------------------- */
static void phase_a_pin_chars(void)
{
    console_print("\r\n--- Phase A: caracterizacion electrica de pines ---\r\n");
    console_print("Lectura con PU=pull-up interno, PD=pull-down interno.\r\n");
    console_print("Interpretacion:\r\n");
    console_print("  PU=1 PD=0 -> linea flotante o alta impedancia (normal)\r\n");
    console_print("  PU=0 PD=0 -> linea FUERTE en bajo (short a GND o driver)\r\n");
    console_print("  PU=1 PD=1 -> linea FUERTE en alto (short a VCC o driver)\r\n");
    console_print("  PU=0 PD=1 -> raro (driver con resistencia justa)\r\n\r\n");

    for (uint8_t i = 0u; i < 4u; ++i) {
        pin_as_input(&kPins[i], GPIO_PULLUP);
        settle_delay();
        uint8_t pu = pin_read(&kPins[i]);

        pin_as_input(&kPins[i], GPIO_PULLDOWN);
        settle_delay();
        uint8_t pd = pin_read(&kPins[i]);

        const char *interp;
        if      (pu == 1u && pd == 0u) interp = "flotante";
        else if (pu == 0u && pd == 0u) interp = "FUERTE EN BAJO";
        else if (pu == 1u && pd == 1u) interp = "FUERTE EN ALTO";
        else                            interp = "raro";

        console_printf("  %s : PU=%u PD=%u  (%s)\r\n",
                       kPins[i].name, (unsigned)pu, (unsigned)pd, interp);
    }
}

/* ------------------------------------------------------------------------- */
/* PHASE A2: cross-talk / shorts entre pines.                                 */
/* Drivea cada pin como output (HIGH y LOW) y lee los otros 3 como input      */
/* con pull-up. Si algun input "sigue" al output, hay short / cruce.          */
/* ------------------------------------------------------------------------- */
static void phase_a2_cross_talk(void)
{
    console_print("\r\n--- Phase A2: cross-talk entre pines ---\r\n");
    console_print("Driver de cada pin a HIGH/LOW, los otros como input PU.\r\n");
    console_print("Si un input sigue al driver, hay short o cruce fisico.\r\n\r\n");

    for (uint8_t drv = 0u; drv < 4u; ++drv) {
        /* Drivea kPins[drv], el resto como input pull-up. */
        for (uint8_t k = 0u; k < 4u; ++k) {
            if (k == drv) {
                pin_as_output(&kPins[k]);
            } else {
                pin_as_input(&kPins[k], GPIO_PULLUP);
            }
        }

        pin_write(&kPins[drv], 1u);
        settle_delay();

        console_printf("  drive %s=HIGH:", kPins[drv].name);
        for (uint8_t k = 0u; k < 4u; ++k) {
            if (k == drv) continue;
            console_printf(" %s=%u", kPins[k].name, (unsigned)pin_read(&kPins[k]));
        }
        console_print("\r\n");

        pin_write(&kPins[drv], 0u);
        settle_delay();

        console_printf("  drive %s=LOW :", kPins[drv].name);
        for (uint8_t k = 0u; k < 4u; ++k) {
            if (k == drv) continue;
            console_printf(" %s=%u", kPins[k].name, (unsigned)pin_read(&kPins[k]));
        }
        console_print("\r\n");
    }
}

/* ------------------------------------------------------------------------- */
/* Bit-bang parametrico (SCK/MOSI/MISO/CS arbitrarios).                       */
/* ------------------------------------------------------------------------- */
static uint8_t bb_xfer(const pin_def_t *sck, const pin_def_t *mosi,
                       const pin_def_t *miso, uint8_t out)
{
    uint8_t in = 0u;
    for (int8_t bit = 7; bit >= 0; --bit) {
        /* Setup MOSI con SCK low. */
        pin_write(mosi, (out >> bit) & 0x01u);
        bb_delay();

        /* Flanco de subida: slave samplea MOSI, master samplea MISO. */
        pin_write(sck, 1u);
        bb_delay();

        if (pin_read(miso)) {
            in |= (uint8_t)(1u << bit);
        }

        /* Flanco de bajada: slave actualiza SDO. */
        pin_write(sck, 0u);
    }
    return in;
}

/* Lee WHO_AM_I (0x0F) por bit-bang con el mapeo dado. */
static uint8_t bb_read_who(const pin_def_t *sck, const pin_def_t *mosi,
                           const pin_def_t *miso, const pin_def_t *cs)
{
    /* Configurar pines. */
    pin_as_output(sck);
    pin_as_output(mosi);
    pin_as_input(miso, GPIO_PULLUP);
    pin_as_output(cs);

    pin_write(sck, 0u);
    pin_write(mosi, 0u);
    pin_write(cs, 1u);
    HAL_Delay(2u);

    /* Transaccion: addr=0x0F | 0x80 (read), luego dummy. */
    pin_write(cs, 0u);
    bb_delay();
    (void)bb_xfer(sck, mosi, miso, (uint8_t)(IMU_REG_WHO_AM_I | 0x80u));
    uint8_t who = bb_xfer(sck, mosi, miso, 0x00u);
    bb_delay();
    pin_write(cs, 1u);

    return who;
}

/* ------------------------------------------------------------------------- */
/* PHASE B: brute-force de las 24 permutaciones de roles.                     */
/* ------------------------------------------------------------------------- */
/* Tabla de las 24 permutaciones de {0,1,2,3} = indices a kPins.
 * Cada fila es (SCK_idx, MOSI_idx, MISO_idx, CS_idx).                        */
static const uint8_t kPerms[24][4] = {
    {0,1,2,3}, {0,1,3,2}, {0,2,1,3}, {0,2,3,1}, {0,3,1,2}, {0,3,2,1},
    {1,0,2,3}, {1,0,3,2}, {1,2,0,3}, {1,2,3,0}, {1,3,0,2}, {1,3,2,0},
    {2,0,1,3}, {2,0,3,1}, {2,1,0,3}, {2,1,3,0}, {2,3,0,1}, {2,3,1,0},
    {3,0,1,2}, {3,0,2,1}, {3,1,0,2}, {3,1,2,0}, {3,2,0,1}, {3,2,1,0},
};

static int phase_b_brute_force(void)
{
    console_print("\r\n--- Phase B: brute-force pinmap (24 permutaciones) ---\r\n");
    console_print("Busca alguna asignacion {SCK,MOSI,MISO,CS} sobre {PA5,PA6,PA7,PC5}\r\n");
    console_print("que devuelva WHO_AM_I=0x70.\r\n\r\n");

    int match = -1;

    for (uint8_t i = 0u; i < 24u; ++i) {
        const pin_def_t *sck  = &kPins[kPerms[i][0]];
        const pin_def_t *mosi = &kPins[kPerms[i][1]];
        const pin_def_t *miso = &kPins[kPerms[i][2]];
        const pin_def_t *cs   = &kPins[kPerms[i][3]];

        uint8_t who = bb_read_who(sck, mosi, miso, cs);

        const char *tag = (who == IMU_WHO_AM_I_VALUE) ? "  <-- MATCH!" : "";

        console_printf("  [%2u/24] SCK=%s MOSI=%s MISO=%s CS=%s  WHO=0x%02X%s\r\n",
                       (unsigned)(i + 1u), sck->name, mosi->name,
                       miso->name, cs->name, (unsigned)who, tag);

        if (who == IMU_WHO_AM_I_VALUE && match < 0) {
            match = (int)i;
            /* No corto: imprimo todas para detectar falsos positivos. */
        }
    }

    return match;
}

/* ------------------------------------------------------------------------- */
/* Entry point publico.                                                       */
/* ------------------------------------------------------------------------- */
void test_imu_diag_run(void)
{
    console_banner("IMU DIAG (bring-up SPI)");

    phase_a_pin_chars();
    phase_a2_cross_talk();
    int match = phase_b_brute_force();

    console_print("\r\n--- Resultado ---\r\n");
    if (match >= 0) {
        const pin_def_t *sck  = &kPins[kPerms[match][0]];
        const pin_def_t *mosi = &kPins[kPerms[match][1]];
        const pin_def_t *miso = &kPins[kPerms[match][2]];
        const pin_def_t *cs   = &kPins[kPerms[match][3]];

        console_printf("MAPPING ENCONTRADO: SCK=%s MOSI=%s MISO=%s CS=%s\r\n",
                       sck->name, mosi->name, miso->name, cs->name);
        console_result(true, "Aplicar este mapping en driver_imu.c");
    } else {
        console_print("Ninguna permutacion devolvio 0x70.\r\n");
        console_print("Revisar: alimentacion del IMU, CS llegando al chip,\r\n");
        console_print("continuidad fisica, IMU danado, o cruce fuera del\r\n");
        console_print("conjunto {PA5,PA6,PA7,PC5}.\r\n");
        console_result(false, "Pinmap no determinado");
    }

    /* Loop infinito para que el test no salga (el LED queda fijo). */
    while (1) {
        HAL_Delay(1000u);
    }
}
