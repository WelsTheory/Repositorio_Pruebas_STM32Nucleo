/*
 * mef.c
 *
 *  Created on: Nov 18, 2020
 *      Author: wels
 */
//Prueba commit

#include "main.h"
#include "sapi.h"
#include "mef.h"
#include "led.h"

extern delay_t MEFSema;
delay_t MEFSemaVerde;
bool_t cuenta = 0;


void MEFSemaInit(void){
	estadoActual = ROJO_;
	delay_ms(&MEFSemaVerde,3000);
}

void MEFSemafActualizar(void){
	switch(estadoActual){
		case ROJO_:
			OnLED(ROJO);
			OffLED(AMARILLO);
			OffLED(VERDE);
			if(delayRead(&MEFSema)){
				if(cuenta >= 4){
					estadoActual = ROJO_AMARILLO;
					cuenta = 0;
				}
				else{
					cuenta++;
				}
			}
		break;
		case ROJO_AMARILLO:
			OnLED(ROJO);
			OnLED(AMARILLO);
			OffLED(VERDE);
			if(delayRead(&MEFSema)){
				if(cuenta >= 1){
					estadoActual = VERDE_;
					cuenta = 0;
				}
				else{
					cuenta++;
				}
			}
		break;
		case VERDE_:
			OffLED(ROJO);
			OffLED(AMARILLO);
			OnLED(VERDE);
			if(delayRead(&MEFSemaVerde)){
				estadoActual = AMARILLO_;
				/*if(cuenta >= 6){
					estadoActual = AMARILLO_;
					cuenta = 0;
				}
				else{
					cuenta++;
				}*/
			}
		break;
		case AMARILLO_:
			OffLED(ROJO);
			OnLED(AMARILLO);
			OffLED(VERDE);
			if(delayRead(&MEFSema)){
				if(cuenta >= 1){
					estadoActual = ROJO_;
					cuenta = 0;
				}
				else{
					cuenta++;
				}
			}
		break;
		default:
			MEFSemaInit();
		break;
	}
}

void buttonPressed(void){
	OnLED(VERDE);
}

void buttonReleased(void){
	OffLED(VERDE);
}

