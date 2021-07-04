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


//Estados LOW y True
#ifndef LOW
#define LOW 	0
#endif

#ifndef TRUE
#define TRUE	1
#endif

//Estados lógicos TRUE y FALSE
#ifndef	FALSE
#define FALSE	0
#endif

#ifndef	TRUE
#define TRUE	(!FALSE)
#endif

//Variables bool_t y tick_t
typedef uint8_t bool_t;
typedef uint16_t tick_t;

//Declaración de estructuras delay_t
typedef struct{
	tick_t startTime;
	tick_t duration;
	bool_t running;
}delay_t;

UART_HandleTypeDef huart2;

//Delay No Bloqueante
/*
 * -> void delay_ms(delay_t * delay,tick_t duration) : Función para crear el delay y la duración
 * -> bool_t delayRead(delay_t * delay) : Lee si el retardo se efectuó
 * */
void delay_ms(delay_t * delay,tick_t duration);
bool_t delayRead(delay_t * delay);

//Delay Bloqueante
/*
 * -> void tickRead(void) : Retorna el valor tickCounter del Callback Timer 11
 * -> void tickWrite(uint16_t ticks) : Escribe el valor tick en tickCounter
 * -> void delay(uint16_t ticks) : Función retardo
 * */
uint16_t tickRead(void);
void tickWrite(uint16_t ticks);
void delay(uint16_t ticks);

// Función de inicialización
void boardInit(void);

#endif /* INC_SAPI_H_ */
