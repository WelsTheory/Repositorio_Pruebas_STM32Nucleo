/*
 * mef.h
 *
 *  Created on: Nov 18, 2020
 *      Author: wels
 */

#ifndef INC_MEF_H_
#define INC_MEF_H_

typedef enum{
	ROJO_,
	ROJO_AMARILLO,
	AMARILLO_,
	VERDE_
} MEFButton_t;

MEFButton_t estadoActual;

void MEFSemaInit(void);
void MEFSemafActualizar(void);
void buttonPressed(void);
void buttonReleased(void);

#endif /* INC_MEF_H_ */
