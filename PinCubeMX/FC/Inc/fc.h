/**
 ******************************************************************************
 * @file    fc.h
 * @brief   Flight controller — punto de entrada del firmware de vuelo.
 ******************************************************************************
 */

#ifndef FC_INC_FC_H_
#define FC_INC_FC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Inicializa sensores y entra al bucle principal. No retorna. */
void fc_run(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_INC_FC_H_ */
