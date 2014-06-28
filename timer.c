#define F_CPU 16000000

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "timer.h"

void timer_init()
{
	/*This is the timer which controls the rate at which
	the GPS data is updated in the code. It is set so that
	an interrupt occurs once every minute or so, and  the
	interrupt sets a flag which is constantly polled*/
	TCCR1A = 0x00; 
	TCCR1B = 0x0C; //Prescale clock by 256, set timer1 to CTC mode for OCR1A
	TIMSK1 = 0x02; //enable compare match 1a
	TIFR1  = 0x02;  //clear timer overflow flag 
	OCR1AH = 0b00000100; //1250. This corresponds to 50Hz
	OCR1AL = 0b11100010;
	return;
}

