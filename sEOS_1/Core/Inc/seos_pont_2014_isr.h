/*
 * seos_pont_2014_isr.h
 *
 *  Created on: Dec 2, 2020
 *      Author: wels
 */

#ifndef INC_SEOS_PONT_2014_ISR_H_
#define INC_SEOS_PONT_2014_ISR_H_

// FUNCION que inicializa el planificador de tareas.
void schedulerInit( void );

// FUNCION que inicializa la interrupcion que ejecuta el planificador de
// tareas.
void schedulerStart( tick_t );

// FUNCION que contiene el planificador de tareas.
void schedulerUpdate( void *ptr );

#endif /* INC_SEOS_PONT_2014_ISR_H_ */
