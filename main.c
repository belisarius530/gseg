#define F_CPU 16000000UL

#include <util/delay.h>
//#include <stdio.h>
#include <avr/io.h>
//#include <math.h>
#include <avr/interrupt.h>

#include "twi.h"
#include "uart.h"
#include "imu.h"
#include "timer.h"
#include "spi.h"
#include "pid.h"


volatile uint8_t timer_flag = 0;

int main(void)
{
	int16_t x_accl = 0;
	int16_t y_accl = 0;
	int16_t z_accl = 0;
	int16_t x_gyro = 0;
	int16_t y_gyro = 0;
	int16_t z_gyro = 0;
	uint16_t count = 0;
	
	twi_init();
	usart_init();
	timer_init();
	accl_init();
	gyro_init();
	spi_init();
	sei();
	
	float temp_val = 0.0;
	
	while(1)
	{
		if(timer_flag > 0)
		{

			accl_position(&x_accl, &y_accl, &z_accl);
		    gyro_position(&x_gyro, &y_gyro, &z_gyro);
			temp_val =  pid( 0, compute_pitch(.5, y_accl, z_accl, x_gyro) );
			spi_write_dac( 2048 + (int16_t)(1300*temp_val) );
			if(!(count++%50))
			{
				usart_send( (int8_t)(57.0*temp_val) );
				//spi_write_dac(2048 + (int16_t)(1300*temp_val) );
			}
      
			timer_flag = 0;
		}
/*		_delay_ms(500);
		spi_write_dac(2048);
		_delay_ms(500);
		spi_write_dac(0);*/
	}
	return 0;
}
	

ISR(TIMER1_COMPA_vect)
{
	timer_flag = 1;
}

/*
#include "adc.h"
#include "uart.h"
#include "spi.h" 

#include "accl.h"
#include "gyro.h" 
#include "imu.h"

volatile uint8_t timer_flag = 0;

int main(void)
{
	double angle = 0;
	uint16_t DLT = 0;
    uint8_t fifty_hz_to_one_hz = 0;

    // Variables associated with the control kernel.
    int yint = 0;
	int yint_prev = 0;
	int error_prev = 0;
	int control = 0;
	int error = 0;
		
	usart_init();
	timer_init();
	spi_init_bitbang();//This interfaces with the gyro.
	accl_init(); 
	spi_init();//This Drives the DAC.
            
	sei();

   	while(1)
	{ 
 	    if(timer_flag > 0)
		{
		    
					
			//spi_write_dac(2000);
		    angle = imu_angle( angle, (.05F) );
            //fifty_hz_to_one_hz++;
    		//if (fifty_hz_to_one_hz == 49) 
			error = angle;
			//{
			yint = yint_prev + KINT*error;
		    control =  (KDER + KPROP)*error - KDER*(error_prev) + yint;

		    yint_prev = yint;
		    error_prev = error;
			////    if(angle <0)
			////	{
			////	    DLT = -angle*4096/90;
		   	////	}
			////	else
			////	{
			////	    DLT = angle*4096/90;
			////	}
               // usart_send(fifty_hz_to_one_hz);
			//	fifty_hz_to_one_hz = 0;
			//}

			
			spi_write_dac(control);
			timer_flag = 0;
			//spi_write_dac(0);
		}
	}
    return 0;
}

ISR(TIMER1_COMPA_vect)
{
   	timer_flag = 1;
}	
*/