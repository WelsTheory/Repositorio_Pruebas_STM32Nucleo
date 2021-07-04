#include "main.h"
#include "sapi.h"
#include "App_Uart.h"
#include "App_Mef.h"

//Tiempos para modo de operación
#define TIEMPO_NORMAL     		500    // ms
#define TIEMPO_LENTO			    2000  // ms
#define TIEMPO_RAPIDO			    100    // ms

//Tiempo para rebote pulsadores
#define TIEMPO_BOTON			    40    // ms

//Variables para retardos
delay_t MEF_Normal;
delay_t MEF_Lento;
delay_t MEF_Rapido;
delay_t MEF_Boton;

int main(void)
{
	boardInit(); //Configuración Inicial STM32
	delay_ms(&MEF_Normal,TIEMPO_NORMAL);
	delay_ms(&MEF_Lento,TIEMPO_LENTO);
	delay_ms(&MEF_Rapido,TIEMPO_RAPIDO);
	delay_ms(&MEF_Boton,TIEMPO_BOTON);
	MEFEstadoInit(); //Maquina de sstado comienza con en el estado Inicial
	while (1)
	{
		 //Actualización Máquinas de estado
		MEFEstadoActualizar();
	}

}
