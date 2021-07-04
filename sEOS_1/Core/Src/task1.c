/*
 * task1.c
 *
 *  Created on: Dec 2, 2020
 *      Author: wels
 */
#include "task1.h"
#include "led.h"

// Función que se ejecuta periodicamente
void task1_Update(void* taskParam){
	OnLED(ROJO);
	OffLED(AMARILLO);
	OffLED(VERDE);
}


