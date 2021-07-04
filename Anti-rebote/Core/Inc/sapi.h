/*
 * sapi.h
 *
 *  Created on: Nov 11, 2020
 *      Author: wels
 */
#ifndef INC_SAPI_H_
#define INC_SAPI_H_

#include "stdint.h"
#include "main.h"
#include "stdio.h"
//#include "sapi_gpio.h"
//#include "semaforo.h"


//Estados
#ifndef LOW
#define LOW 	0
#endif

#ifndef TRUE
#define TRUE	1
#endif

//Estados lógicos
#ifndef	FALSE
#define FALSE	0
#endif

#ifndef	TRUE
#define TRUE	(!FALSE)
#endif

//Variable
typedef uint8_t bool_t;
typedef uint16_t tick_t;

//Funciones Delay
typedef struct{
	tick_t startTime;
	tick_t duration;
	bool_t running;
}delay_t;

void delay_ms(delay_t * delay,tick_t duration);
bool_t delayRead(delay_t * delay);
void delayWrite(delay_t * delay, tick_t duration);

void boardInit(void);
uint16_t tickRead(void);
void tickWrite(uint16_t ticks);
void delay(uint16_t ticks);

#endif /* INC_SAPI_H_ */
