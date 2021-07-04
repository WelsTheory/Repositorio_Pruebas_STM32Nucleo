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

//Funciones
void boardInit(void);
uint16_t tickRead(void);
void tickWrite(uint16_t ticks);

#endif /* INC_SAPI_H_ */
