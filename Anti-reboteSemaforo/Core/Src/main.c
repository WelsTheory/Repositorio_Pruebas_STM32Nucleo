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
	MEFSemaInit();
	delay_ms(&MEFSema,500);
	while (1)
	{
		MEFSemafActualizar();
		//Desconectado();
		//MEFButtonActualizar();
	}

}
