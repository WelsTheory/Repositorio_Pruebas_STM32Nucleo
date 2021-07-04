/*
 * App_uart.h
 *
 *  Created on: Dec 3, 2020
 *      Author: wels
 */

#ifndef INC_APP_UART_H_
#define INC_APP_UART_H_

#include "stdint.h"

/*
 * -> void UART_String(const char* str, uint16_t sz) : Función para enviar una cadena de datos a través de la UART
 * str = cadena
 * sz = tamaño de la cadena
*/
void UART_String(const char* str, uint16_t sz); // .


#endif /* INC_APP_UART_H_ */
