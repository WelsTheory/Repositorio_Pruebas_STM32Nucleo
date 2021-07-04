/*
 * App_uart.c
 *
 *  Created on: Dec 3, 2020
 *      Author: wels
 */
#include "App_uart.h"
#include "main.h"
#include "sapi.h"

//UART_HandleTypeDef &huart2;

//void UART_Write(const uint8_t data); // Función para enviar un dato a través de la UART.
//void UART_String(const char* str); // Función para enviar una cadena de datos a través de la UART.

void UART_String(const char* str, uint16_t sz){
	HAL_UART_Transmit(&huart2, str, sz, HAL_MAX_DELAY);
}

void UART_Write(void){
	uint8_t otro= 0x30;
	HAL_UART_Transmit_IT(&huart2, otro, 1);
}
