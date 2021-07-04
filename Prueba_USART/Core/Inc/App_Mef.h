/*
 * App_mef.h
 *
 *  Created on: Dec 3, 2020
 *      Author: wels
 */

#ifndef INC_APP_MEF_H_
#define INC_APP_MEF_H_

// Estructura MEFEstado
typedef enum{
	ESTADO_NORMAL,
	ESTADO_LENTO,
	ESTADO_RAPIDO
}MEFEstado_t;

MEFEstado_t estadoActual;

void MEFEstadoInit(void); // Función para iniciar el primer estado
void MEFEstadoActualizar(void); //Función de lectura de pulsadores y cambio de estado.


#endif /* INC_APP_MEF_H_ */
