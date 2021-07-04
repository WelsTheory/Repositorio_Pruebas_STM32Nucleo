/*
 * seos_pont_2014_scheduler.h
 *
 *  Created on: Dec 2, 2020
 *      Author: wels
 */

#ifndef INC_SEOS_PONT_2014_SCHEDULER_H_
#define INC_SEOS_PONT_2014_SCHEDULER_H_

/* Store in DATA area, if possible, for rapid access.
   Total memory per task is 7 bytes. */
typedef struct{
   // Pointer to the task (must be a 'void (void)' function)
   callBackFuncPtr_t pTask;  // void (* pTask)(void);
   // Delay (ticks) until the function will (next) be run
   // - see schedulerAddTask() for further details
   void* taskParam; // Parametro de tarea
   // Delay (ticks) until the function will (next) be run
   // - see schedulerAddTask() for further details
   int32_t delay;
   // Interval (ticks) between subsequent runs.
   // - see schedulerAddTask() for further details
   int32_t period;
   // Incremented (by scheduler) when task is due to execute
   int32_t runMe;
} sTask_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// FUNCION que contiene el despachador de tareas.
void schedulerDispatchTasks( void );

// FUNCION que añade una tarea al planificador.
int32_t schedulerAddTask( callBackFuncPtr_t pFunction, //void (* pFunction)(void),
                          void* TASKPARAM,
                          const int32_t DELAY,
                          const int32_t PERIOD
                        );

// FUNCION que remueve una tarea del planificador.
int8_t schedulerDeleteTask( int32_t taskIndex );

// FUNCION que reporta el estado del sistema.
void schedulerReportStatus( void );

#endif /* INC_SEOS_PONT_2014_SCHEDULER_H_ */
