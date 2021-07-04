/*
 * semaforo.c
 *
 *  Created on: Nov 4, 2020
 *      Author: wels
 */
#include "semaforo.h"
#include "sapi.h"
#include "led.h"

bool_t sec;
extern delay_t SemaforoTiempo;

void Normal(void){
	apagarLeds();
	OnLED(ROJO);
	//delayRead(&MEFBoton)
	delay(1000);
	apagarLeds();
	OnLED(AMARILLO);
	delay(500);
	apagarLeds();
	OnLED(VERDE);
	delay(200);
	apagarLeds();
}

void Desconectado(void){
	OnLED(AMARILLO);
	delay(500);
	OffLED(AMARILLO);
	delay(500);
	//apagarLeds();
}

void Alarma(void){
	OnLED(AMARILLO);
	OnLED(ROJO);
	delay(1000);
	OffLED(AMARILLO);
	OffLED(ROJO);
	delay(1000);
}

void leerTeclas(void){
	if(!HAL_GPIO_ReadPin(GPIOC, B1_Pin)){
		sec = 0;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, B2_Pin)){
		sec = 1;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, B3_Pin)){
		sec = 2;
	}
}

void Estado(void){
	leerTeclas();
	if(sec == 0){
		Normal();
	}
	if(sec == 1){
		Desconectado();
	}
	if(sec == 2){
		Alarma();
	}
}

