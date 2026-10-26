/**
 ******************************************************************************
 * @file    bal_link.h
 * @brief   Enlace PC <-> balancin por USB CDC: comandos ASCII de una linea
 *          (terminados en \r o \n, mayusculas o minusculas).
 *
 *            A | ARM       armar (balancea hacia la referencia actual)
 *            D | STOP      desarmar (motores a 0)
 *            R <grados>    referencia de angulo, antihorario + visto desde
 *                          atras del FC. Ej: "R 10", "R-5.5". Boot = 0.
 *            T <pct>       throttle base comun, 0..20 %. Ej: "T 9.5"
 *            Z             con el balancin NIVELADO (nivel de burbuja) y
 *                          desarmado: toma el angulo actual como 0
 *                          (corrige el montaje de la IMU, se pierde al
 *                          reiniciar; dejarlo fijo en BAL_ANGLE_TRIM_DEG)
 *            ?             linea de estado
 *
 *          Respuestas: "OK ..." / "ERR ..." / "ST ...". Se mandan sin
 *          bloquear (console_try_print): si el host no lee, se pierden.
 ******************************************************************************
 */

#ifndef BALANCER_BAL_LINK_H_
#define BALANCER_BAL_LINK_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BAL_CMD_NONE = 0,
    BAL_CMD_ARM,
    BAL_CMD_STOP,
    BAL_CMD_REF,
    BAL_CMD_THROTTLE,
    BAL_CMD_STATUS,
    BAL_CMD_ZERO,
    BAL_CMD_INVALID
} bal_cmd_type_t;

typedef struct {
    bal_cmd_type_t type;
    float          value;   /* R: grados, T: porcentaje */
} bal_cmd_t;

void bal_link_init(void);

/* Consume los bytes recibidos. true cuando completo una linea (cmd valido o
 * BAL_CMD_INVALID). Llamar en un loop hasta que devuelva false.           */
bool bal_link_poll(bal_cmd_t *cmd);

/* printf no bloqueante hacia la PC (max 255 chars, sin floats). */
void bal_link_reply(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* BALANCER_BAL_LINK_H_ */
