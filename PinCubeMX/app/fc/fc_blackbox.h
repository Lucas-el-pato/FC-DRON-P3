/**
 ******************************************************************************
 * @file    fc_blackbox.h
 * @brief   Blackbox propio: registro del lazo PID en la microSD para ajustar
 *          los PID en la PC (respuesta al escalon, ruido), como el blackbox
 *          de Betaflight pero con formato propio.
 *
 *          Flujo:
 *            boot      monta la SD y reserva BBnnn.BBL contiguo (bloqueante)
 *            armado    cada FC_BLACKBOX_RATE_DENOM PIDs se copia un registro
 *                      de 95 B a un ring en RAM (~1 us, nunca bloquea)
 *            vuelo     BB_WRITE manda bloques de 4 KB por DMA directo a los
 *                      sectores del archivo; FatFs no se usa en vuelo
 *            desarme   vacia el ring, recorta el archivo al tamano real y
 *                      reserva el siguiente (bloqueante, solo desarmado)
 *
 *          Archivo: cabecera de texto de FC_BB_HEADER_LEN bytes (version,
 *          tasas, ganancias y lista de campos "nombre:tipo:escala") y luego
 *          registros bb_record_t. Decoder: tools/blackbox/bbl_decode.py.
 ******************************************************************************
 */

#ifndef FC_FC_BLACKBOX_H_
#define FC_FC_BLACKBOX_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 0 = sin blackbox (no toca la SD). */
#ifndef FC_ENABLE_BLACKBOX
#define FC_ENABLE_BLACKBOX       1
#endif

/* 1 registro cada N ejecuciones del PID: 2 kHz / 2 = 1 kHz (95 KB/s). */
#ifndef FC_BLACKBOX_RATE_DENOM
#define FC_BLACKBOX_RATE_DENOM   2u
#endif

/* Tamano reservado por archivo. 64 MB a 95 KB/s = ~11 min armado. */
#ifndef FC_BLACKBOX_FILE_MB
#define FC_BLACKBOX_FILE_MB      64u
#endif

#define FC_BB_FORMAT_VERSION     3u   /* 2: eRPM, carga de CPU; 3: notch dinamico */
#define FC_BB_HEADER_LEN         2048u   /* multiplo de 512 */
#define FC_BB_RECORD_LEN         95u

typedef enum {
    BB_STATE_OFF = 0,     /* sin tarjeta / no montada / deshabilitado    */
    BB_STATE_READY,       /* archivo reservado, esperando armado         */
    BB_STATE_LOGGING,     /* armado, grabando                             */
    BB_STATE_FLUSHING,    /* desarmado, vaciando el ring                  */
    BB_STATE_FULL,        /* se lleno el archivo: deja de grabar          */
    BB_STATE_ERROR        /* fallo de escritura: deja de grabar           */
} bb_state_t;

typedef enum {
    BB_ERR_NONE = 0,
    BB_ERR_MOUNT,         /* no monto (sin tarjeta, sin FAT32)            */
    BB_ERR_NOSPACE,       /* menos de 4 MB libres                         */
    BB_ERR_CREATE,        /* no se pudo crear / fragmentado: formatear    */
    BB_ERR_WRITE,         /* error SDIO/DMA en vuelo                      */
    BB_ERR_TIMEOUT,       /* una escritura tardo > 1 s                    */
    BB_ERR_CLOSE          /* no se pudo recortar / cerrar el archivo      */
} bb_error_t;

typedef struct {
    bb_state_t state;
    bb_error_t error;
    uint16_t   file_index;    /* nnn de BBnnn.BBL (0 = ninguno)          */
    uint32_t   bytes;         /* bytes logicos del archivo en curso      */
    uint32_t   records;
    uint32_t   drops;         /* registros perdidos por ring lleno       */
    uint8_t    ring_peak_pct; /* ocupacion maxima del ring en el vuelo   */
} blackbox_status_t;

/* Boot: monta la SD y reserva el primer archivo. Bloqueante (segundos sin
 * tarjeta). Llamar antes de scheduler_run().                            */
void blackbox_init(void);

/* Desde fc_task_pid, despues del mixer, en cada ejecucion del PID. Detecta
 * armado / desarmado y copia el registro al ring. Nunca bloquea.        */
void blackbox_record(void);

/* Tarea lenta rapida (~1 ms): avanza las escrituras DMA. No bloquea.    */
void blackbox_task_write(void);

/* Tarea lenta de archivo (~100 ms): cierra y reserva archivos, solo con
 * el dron desarmado. Bloqueante.                                         */
void blackbox_task_file(void);

const blackbox_status_t *blackbox_status(void);
const char *blackbox_state_name(bb_state_t st);

#ifdef __cplusplus
}
#endif

#endif /* FC_FC_BLACKBOX_H_ */
