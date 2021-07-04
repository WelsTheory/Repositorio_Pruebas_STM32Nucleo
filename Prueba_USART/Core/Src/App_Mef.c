/*
 * App_mef.c
 *
 *  Created on: Dec 3, 2020
 *      Author: wels
 */
#include "main.h"
#include "sapi.h"
#include "App_Mef.h"
#include "led.h"
#include "App_Uart.h"

//Variables de Retardo
extern delay_t MEF_Boton;
extern delay_t MEF_Normal;
extern delay_t MEF_Lento;
extern delay_t MEF_Rapido;

// Cadena de datos a transmitir
const char dato1[15] = "MODO NORMAL\n\r";
const char dato2[14] = "MODO LENTO\n\r";
const char dato3[15] = "MODO RAPIDO\n\r";

//Maquina de sstado comienza con en el estado Inicial
void MEFEstadoInit(void){
	OffLED(VERDE);
	OffLED(ROJO);
	OffLED(AMARILLO);
	estadoActual = ESTADO_NORMAL;
	UART_String(dato1,15);
}

//Actualización Máquinas de estado
void MEFEstadoActualizar(void){

	switch(estadoActual){

	case ESTADO_NORMAL:
		OffLED(VERDE);
		OnLED(ROJO);
		if(delayRead(&MEF_Normal)){
			//Si cumple tiempo de retardo Normal, envia el mensaje "Modo Normal"
			UART_String(dato1,15);
		}

		if(GPIO_Read(TEC1)){
			//Si se presiona TEC1 ingresa al anti-rebote
			if(delayRead(&MEF_Boton)){
				//Cumple tiempo de retardo del anti-rebote
				if(GPIO_Read(TEC1)){
					//Si se presiona TEC1, cambia el estado actual ingresa al "Modo Lento"
					UART_String(dato2,14);
					estadoActual = ESTADO_LENTO;
				}
			}
		}

		break;

	case ESTADO_LENTO:
		OffLED(ROJO);
		OnLED(AMARILLO);
		if(delayRead(&MEF_Lento)){
			//Si cumple tiempo de retardo Lento, envia el mensaje "Modo Lento"
			UART_String(dato2,14);
		}

		if(GPIO_Read(TEC2)){
			//Si se presiona TEC2 ingresa al anti-rebote
			if(delayRead(&MEF_Boton)){
				//Cumple tiempo de retardo del anti-rebote
				if(GPIO_Read(TEC2)){
					//Si se presiona TEC2, cambia el estado actual ingresa al "Modo Rapido"
					UART_String(dato3,15);
					estadoActual = ESTADO_RAPIDO;
				}
			}
		}

		break;

	case ESTADO_RAPIDO:
		OffLED(AMARILLO);
		OnLED(VERDE);
		if(delayRead(&MEF_Rapido)){
			//Si cumple tiempo de retardo Rapido, envia el mensaje "Modo Rapido"
			UART_String(dato3,15);
		}

		if(GPIO_Read(TEC3)){
			//Si se presiona TEC3 ingresa al anti-rebote
			if(delayRead(&MEF_Boton)){
				//Cumple tiempo de retardo del anti-rebote
				if(GPIO_Read(TEC3)){
					//Si se presiona TEC3, cambia el estado actual ingresa al "Modo Normal"
					UART_String(dato1,15);
					estadoActual = ESTADO_NORMAL;
				}
			}
		}

		break;

	default:
		MEFEstadoInit(); //Estado Inicial
		break;
	}
}
