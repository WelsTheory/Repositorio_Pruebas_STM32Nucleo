#include "main.h"
#include "sapi.h"
#include "App_Uart.h"
#include "App_Mef.h"

int main(void)
{
	boardInit(); //Configuración Inicial STM32
	MEFEstadoInit(); //Maquina de sstado comienza con en el estado Inicial
	while (1)
	{
		 //Actualización Máquinas de estado
		MEFEstadoActualizar();
		HAL_Delay(100);
	}

}
