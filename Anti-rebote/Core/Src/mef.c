/*
 * mef.c
 *
 *  Created on: Nov 18, 2020
 *      Author: wels
 */

#include "main.h"
#include "sapi.h"
#include "mef.h"
#include "led.h"

extern delay_t MEFBoton;

void MEFButtonInit(void){
	estadoActual = BUTTON_UP;
}

void MEFButtonActualizar(){
	switch(estadoActual){
		case BUTTON_UP:
			if(!HAL_GPIO_ReadPin(GPIOB, B1_Pin)){
				estadoActual = BUTTON_FALLING;
			}
		break;
		case BUTTON_FALLING:
			if(delayRead(&MEFBoton)){
				if(!HAL_GPIO_ReadPin(GPIOB, B1_Pin)){
					estadoActual = BUTTON_DOWN;
					buttonPressed();
				}
			}
			else{
				estadoActual = BUTTON_UP;
			}
		break;
		case BUTTON_DOWN:
			if(HAL_GPIO_ReadPin(GPIOB, B1_Pin)){
				estadoActual = BUTTON_RISING;
				buttonPressed();
			}
		break;
		case BUTTON_RISING:
			if(delayRead(&MEFBoton)){
				if(HAL_GPIO_ReadPin(GPIOB, B1_Pin)){
					estadoActual = BUTTON_UP;
					buttonReleased();
				}
			}
			else{
				estadoActual = BUTTON_DOWN;
			}
		break;
		default:
			MEFButtonInit();
		break;
	}
}

void buttonPressed(void){
	OnLED(VERDE);
}

void buttonReleased(void){
	OffLED(VERDE);
}

