/**
 ******************************************************************************
 * @file    console.c
 * @brief   Implementacion de la consola debug por USB CDC + LEDs de estado.
 ******************************************************************************
 */

#include "console.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Numero maximo de reintentos cuando CDC_Transmit_FS devuelve USBD_BUSY. */
#define CONSOLE_CDC_MAX_RETRIES 80u
/* Espera (ms) entre reintentos cuando el endpoint esta ocupado.          */
#define CONSOLE_CDC_RETRY_DELAY 2u
/* Timeout total (ms) esperando que el host abra el puerto COM virtual.   */
/* Pasado este tiempo, el test arranca igual (sin salida por consola).    */
#define CONSOLE_PORT_OPEN_TIMEOUT 15000u
/* Periodo (ms) entre chequeos del flag de puerto abierto.                */
#define CONSOLE_PORT_POLL_PERIOD  10u

/* Tamano maximo del buffer para console_printf().                        */
#define CONSOLE_PRINTF_BUFLEN   192u

void console_init(void)
{
    /* Apaga LEDs por las dudas (MX_GPIO_Init los deja en reset). */
    HAL_GPIO_WritePin(led_rojo_GPIO_Port,  led_rojo_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);

    /* Espera activa hasta que el host abra el puerto COM (DTR asserted).
     * Si el usuario abre PuTTY despues del reset, aca quedamos esperando.
     * Con timeout para no colgar el test si no hay host conectado.        */
    for (uint32_t waited = 0u;
         (waited < CONSOLE_PORT_OPEN_TIMEOUT) && (CDC_IsPortOpen() == 0u);
         waited += CONSOLE_PORT_POLL_PERIOD) {
        HAL_Delay(CONSOLE_PORT_POLL_PERIOD);
    }

    /* Pequena espera extra para que el driver CDC del host termine de
     * configurar el endpoint antes del primer transmit.                   */
    if (CDC_IsPortOpen() != 0u) {
        HAL_Delay(100u);
    }

    /* Saludo. */
    console_print("\r\n");
    console_print("===========================================\r\n");
    console_print(" Consola de test - FC Dron STM32F405RGTx\r\n");
    console_print("===========================================\r\n");
}

void console_print(const char *s)
{
    if (s == NULL) {
        return;
    }

    uint16_t len = (uint16_t)strlen(s);
    if (len == 0u) {
        return;
    }

    /* Si el host no tiene abierto el puerto COM, descartamos sin tocar el
     * endpoint USB: evita que TxState quede atascado en 1 esperando un IN
     * token que nunca llegara, y permite que las prints posteriores (cuando
     * el usuario abra PuTTY) funcionen sin lag.                            */
    if (CDC_IsPortOpen() == 0u) {
        return;
    }

    for (uint32_t i = 0u; i < CONSOLE_CDC_MAX_RETRIES; ++i) {
        if (CDC_IsPortOpen() == 0u) {
            /* El host cerro el puerto a mitad de la espera. Descartamos.   */
            return;
        }
        uint8_t st = CDC_Transmit_FS((uint8_t *)s, len);
        if (st == USBD_OK) {
            return;
        }
        HAL_Delay(CONSOLE_CDC_RETRY_DELAY);
    }
    /* Si tras todos los reintentos sigue ocupado, descartamos. No bloquea
     * la ejecucion del test (mejor seguir testeando hardware que colgarse). */
}

void console_printf(const char *fmt, ...)
{
    char buf[CONSOLE_PRINTF_BUFLEN];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }
    if ((size_t)n >= sizeof(buf)) {
        /* Trunca: asegura terminacion en NUL. */
        buf[sizeof(buf) - 1u] = '\0';
    }
    console_print(buf);
}

/* NOTA: las etiquetas led_verde_* / led_rojo_* del .ioc estan invertidas
 * respecto al cableado real de la placa, asi que aca apuntamos pass al pin
 * etiquetado "rojo" y fail al etiquetado "verde". No se toca el .ioc para
 * evitar regenerar GPIO y romper otros pines.                                */
void console_led_pass(void)
{
    HAL_GPIO_WritePin(led_rojo_GPIO_Port,  led_rojo_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);
}

void console_led_fail(void)
{
    HAL_GPIO_WritePin(led_rojo_GPIO_Port,  led_rojo_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_SET);
}

void console_led_off(void)
{
    HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led_rojo_GPIO_Port,  led_rojo_Pin,  GPIO_PIN_RESET);
}

void console_led_set(bool pass)
{
    if (pass) {
        console_led_pass();
    } else {
        console_led_fail();
    }
}

void console_banner(const char *test_name)
{
    if (test_name == NULL) {
        test_name = "(unknown)";
    }
    console_printf("\r\n=== Test %s ===\r\n", test_name);
}

void console_result(bool ok, const char *detail)
{
    if (detail == NULL) {
        detail = "";
    }
    if (ok) {
        console_printf("[PASS] %s\r\n", detail);
    } else {
        console_printf("[FAIL] %s\r\n", detail);
    }
    console_led_set(ok);
}
