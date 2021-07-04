/*
 * mef.h
 *
 *  Created on: Nov 18, 2020
 *      Author: wels
 */

#ifndef INC_MEF_H_
#define INC_MEF_H_

typedef enum{
	BUTTON_UP,
	BUTTON_FALLING,
	BUTTON_RISING,
	BUTTON_DOWN
} MEFButton_t;

MEFButton_t estadoActual;

void MEFButtonInit(void);
void MEFButtonActualizar(void);
void buttonPressed(void);
void buttonReleased(void);

#endif /* INC_MEF_H_ */
