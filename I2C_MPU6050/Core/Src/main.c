#include "main.h"
#include "sapi.h"
#include "imu_mpu6050.h"

char buf[4];
uint8_t dato = 0;

int main(void)
{

	boardInit(); //Configuración Inicial STM32
	//HAL_I2C_MspInit(hi2c);
	MPU6050_Init();

	  while (1)
	  {
		  dato++;
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */

		  // read the Accelerometer and Gyro values

		  MPU6050_Read_Accel();
		  //MPU6050_Read_Gyro();

		  // print the Acceleration and Gyro values on the LCD 20x4

		  /*sprintf (buf, "%.2f", Ax);


		  sprintf (buf, "%.2f", Ay);

		  sprintf (buf, "%.2f", Az);

		  sprintf (buf, "%.2f", Gx);

		  sprintf (buf, "%.2f", Gy);

		  sprintf (buf, "%.2f", Gz);*/

		  printf("cuenta = %d",dato);

		  HAL_Delay (250);  // wait for a while
	  }

}
