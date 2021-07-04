/*
 * led.h
 *
 *  Created on: Nov 4, 2020
 *      Author: wels
 */

#ifndef INC_LED_H_
#define INC_LED_H_

typedef enum{
	VERDE, AMARILLO, ROJO, NA,
	TEC1, TEC2, TEC3, TEC4,// TEC1,
}gpioMap_led;

void OnLED(gpioMap_led led);
void OffLED(gpioMap_led led);

void apagarLeds();
void leerTeclas();
void activarSecuencia();


#endif /* INC_LED_H_ */
