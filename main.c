#define F_CPU 16000000UL
#define KPROP 2.5
#define KDER 5
#define KINT 1

#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>

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
