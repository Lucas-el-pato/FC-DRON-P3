/**
 ******************************************************************************
 * @file    console.h
 * @brief   Consola de debug por USB CDC (impresion estilo printf) y helpers
 *          de LED de diagnostico (verde = PASS, rojo = FAIL).
 *
 *          Para visualizar la salida en PUTTY conectar el cable USB del MCU
 *          (puerto OTG_FS) a la PC y abrir el puerto COM virtual a cualquier
 *          baud (USB CDC ignora el baudrate). Recomendado: 115200, 8N1.
 ******************************************************************************
 */

#ifndef TESTS_INC_CONSOLE_H_
#define TESTS_INC_CONSOLE_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Inicializa la consola. Espera (con timeout de 15 s) a que el host abra el  */
/* puerto COM virtual (DTR asserted). Asi el usuario puede abrir PuTTY        */
/* despues del reset y no perder la salida del test.                          */
/* ------------------------------------------------------------------------- */
void console_init(void);

/* ------------------------------------------------------------------------- */
/* Envia una cadena por USB CDC (con reintentos por USBD_BUSY).               */
/* ------------------------------------------------------------------------- */
void console_print(const char *s);

/* ------------------------------------------------------------------------- */
/* Impresion estilo printf (max 192 chars por llamada).                       */
/* ------------------------------------------------------------------------- */
void console_printf(const char *fmt, ...);

/* ------------------------------------------------------------------------- */
/* Helpers de LED de diagnostico.                                             */
/*   console_led_pass(): verde ON, rojo OFF.                                  */
/*   console_led_fail(): rojo ON, verde OFF.                                  */
/*   console_led_off() : ambos OFF.                                           */
/*   console_led_set(pass): selecciona segun bool.                            */
/* ------------------------------------------------------------------------- */
void console_led_pass(void);
void console_led_fail(void);
void console_led_off(void);
void console_led_set(bool pass);

/* ------------------------------------------------------------------------- */
/* Banner de inicio de test: imprime una linea con el nombre.                 */
/* ------------------------------------------------------------------------- */
void console_banner(const char *test_name);

/* ------------------------------------------------------------------------- */
/* Imprime resultado final: [PASS] o [FAIL] segun ok.                         */
/* ------------------------------------------------------------------------- */
void console_result(bool ok, const char *detail);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_CONSOLE_H_ */
