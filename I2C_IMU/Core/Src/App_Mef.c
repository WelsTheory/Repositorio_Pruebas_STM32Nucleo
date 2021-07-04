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
		//OffLED(VERDE);
		OnLED(A_BOARD);
		if(delayRead(&MEF_Normal)){
			//Si cumple tiempo de retardo Normal, envia el mensaje "Modo Normal"
			//UART_String(dato1,15);
			estadoActual = ESTADO_LENTO;
		}
		break;

	case ESTADO_LENTO:
		OffLED(A_BOARD);
//		OffLED(ROJO);
//		OnLED(AMARILLO);
		if(delayRead(&MEF_Lento)){
			//Si cumple tiempo de retardo Lento, envia el mensaje "Modo Lento"
			//UART_String(dato2,14);
			estadoActual = ESTADO_RAPIDO;
		}
		break;

	case ESTADO_RAPIDO:
		//OnLED(A_BOARD);
		if(delayRead(&MEF_Rapido)){
			//Si cumple tiempo de retardo Rapido, envia el mensaje "Modo Rapido"
			//UART_String(dato3,15);
			Toggle(A_BOARD);
			//estadoActual = ESTADO_NORMAL;
		}
		break;

	default:
		MEFEstadoInit(); //Estado Inicial
		break;
	}
}
