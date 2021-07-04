

#include "main.h"
#include "sapi.h"
#include "sapi_gpio.h"




int main(void)
{
	boardInit();
	while (TRUE)
	{
		activarSecuencia();
	}

}


