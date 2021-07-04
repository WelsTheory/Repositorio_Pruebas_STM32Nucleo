#include "main.h"
#include "sapi.h"
#include "led.h"
#include "semaforo.h"
#include "mef.h"

delay_t MEFBoton;
delay_t SemaforoTiempo;

int main(void)
{
	boardInit();
	MEFButtonInit();
	delay_ms(&MEFBoton,40);
	while (1)
	{
		//Desconectado();
		MEFButtonActualizar();
	}

}
