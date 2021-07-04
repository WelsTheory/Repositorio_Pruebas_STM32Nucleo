#include "main.h"
#include "sapi.h"
#include "led.h"
#include "semaforo.h"
#include "mef.h"

delay_t MEFSema;
delay_t SemaforoTiempo;

int main(void)
{
	boardInit();

	// FUNCION que inicializa el planificador de tareas
	schedulerInit();

	// Se agrega la tarea tarea1 al planificador
	schedulerAddTask( task1_Update, // funcion de tarea a agregar
			0,            // parametro de la tarea
			0,            // offset de ejecucion en ticks
			100           // periodicidad de ejecucion en ticks
	);

	MEFSemaInit();
	delay_ms(&MEFSema,500);
	while (1)
	{
		MEFSemafActualizar();
		//Desconectado();
		//MEFButtonActualizar();
	}

}
