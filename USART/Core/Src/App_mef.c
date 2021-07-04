/*
 * App_mef.c
 *
 *  Created on: Dec 3, 2020
 *      Author: wels
 */


#include "main.h"
#include "sapi.h"
#include "App_mef.h"
#include "led.h"
#include "App_uart.h"

extern delay_t MEFEst;
extern delay_t MEFBoton;

bool_t cuenta = 0;

uint8_t dato1[15] = "MODO NORMAL\n\r";
uint8_t dato2[14] = "MODO LENTO\n\r";
uint8_t dato3[15] = "MODO RAPIDO\n\r";

//ESTDO_NORMAL, ESTADO_LENTO, ESTADO_RAPIDO
void MEFEstadoInit(void){
	estadoActual = ESTADO_NORMAL;
	UART_String(dato1,15);
}

void MEFEstadoActualizar(void){
	switch(estadoActual){
	case ESTADO_NORMAL:
		OnLED(ROJO);
		OffLED(VERDE);
		if(delayRead(&MEFEst)){
			if(cuenta >= 5){
				UART_String(dato1,15);
				cuenta = 0;
			}
			else{
				cuenta++;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOB, B1_Pin)){
			if(delayRead(&MEFBoton)){
				if(!HAL_GPIO_ReadPin(GPIOB, B1_Pin)){
					UART_String(dato2,14);
					estadoActual = ESTADO_LENTO;
				}
			}
		}
		break;
	case ESTADO_LENTO:
		OnLED(AMARILLO);
		OffLED(ROJO);
		if(delayRead(&MEFEst)){
			if(cuenta >= 20){
				UART_String(dato2,14);
				cuenta = 0;
			}
			else{
				cuenta++;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOB, B2_Pin)){
			if(delayRead(&MEFBoton)){
				if(!HAL_GPIO_ReadPin(GPIOB, B2_Pin)){
					UART_String(dato3,15);
					estadoActual = ESTADO_RAPIDO;
				}
			}
		}
		break;
	case ESTADO_RAPIDO:
		OnLED(VERDE);
		OffLED(AMARILLO);
		if(delayRead(&MEFEst)){
			if(cuenta >= 1){
				UART_String(dato3,15);
				cuenta = 0;
			}
			else{
				cuenta++;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOC, B0_Pin)){
			if(delayRead(&MEFBoton)){
				if(!HAL_GPIO_ReadPin(GPIOC, B0_Pin)){
					UART_String(dato1,15);
					estadoActual = ESTADO_NORMAL;
				}
			}
		}
		break;
	default:
		MEFEstadoInit();
		break;
	}
}
