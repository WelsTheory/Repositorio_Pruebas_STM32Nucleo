/*
 * sapi_gpio.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Wels
 */

#ifndef INC_SAPI_GPIO_H_
#define INC_SAPI_GPIO_H_

typedef uint8_t bool_t;

typedef enum{
	LED1, LED2, LED3, LED4,
	BT1, BT2, BT3, BT4,// TEC1,
}gpioMap_t;

// FUNCTIONS
bool_t encenderLED(gpioMap_t led);
void apagarLeds();
void leerTeclas();
void activarSecuencia();




#endif /* INC_SAPI_GPIO_H_ */
