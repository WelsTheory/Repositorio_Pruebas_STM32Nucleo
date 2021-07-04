/*
 * sapi.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Wels
 */

#ifndef INC_SAPI_H_
#define INC_SAPI_H_

#include "stdint.h"
#include "main.h"
#include "sapi_gpio.h"


//Estados
#ifndef LOW
#define LOW 	0
#endif

#ifndef TRUE
#define TRUE	1
#endif

//Estados lógicos
#ifndef	FALSE
#define FALSE	0
#endif

#ifndef	TRUE
#define TRUE	(!FALSE)
#endif

//Variable
typedef uint8_t bool_t;

//Funciones
void boardInit(void);



#endif /* INC_SAPI_H_ */
