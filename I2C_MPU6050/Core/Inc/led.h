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
}gpioMap_led;

typedef enum{
	TEC1, TEC2, TEC3, TEC4,
}gpioMap_button;

void OnLED(gpioMap_led led);
void OffLED(gpioMap_led led);
void Toggle(gpioMap_led led);

bool_t GPIO_Read(gpioMap_button tec);

void apagarLeds();
void leerTeclas();
void activarSecuencia();


#endif /* INC_LED_H_ */
