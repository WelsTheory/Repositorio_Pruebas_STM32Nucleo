#include "main.h"
#include "sapi.h"
#include "led.h"
#include "semaforo.h"

int main(void)
{
	boardInit();
	while (1)
	{
		Normal();
		/*tiempo = tickRead();
		if(tiempo >= 100){
			Toggle(VERDE);
			tickWrite(0);
		}*/

	}

}
