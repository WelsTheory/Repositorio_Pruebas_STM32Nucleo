/*
 * App_uart.c
 *
 *  Created on: Dec 3, 2020
 *      Author: wels
 */
#include "App_uart.h"
#include "main.h"
#include "sapi.h"

void UART_String(const char* str, uint16_t sz){
	HAL_UART_Transmit(&huart2, str, sz, HAL_MAX_DELAY);
}
