#include "main.h"
#include "sapi.h"
#include "App_uart.h"

delay_t MEFEst;
delay_t MEFBoton;

int main(void)
{
	boardInit();
	delay_ms(&MEFEst,100);
	delay_ms(&MEFBoton,40);
	MEFEstadoInit();
	while (1)
	{
		MEFEstadoActualizar();
	}

}
