/***************************************************************
* Filename: UART.C
* Authors: Jon Fish, Thomas Stobaugh, Unknown 
* Description: This file contains the functions needed to use
* the UART/USART module on the ATmega 328.
***************************************************************/
#include <stdlib.h> 
#include <avr/io.h> 
#include "uart.h"
#include <math.h>

/***************************************************************
* This Function prompts the UART to transmit a byte.
***************************************************************/
void usart_init(void)
{
	uint32_t ubrr = (CLK_SPEED/16UL)/BAUD-1; // calculates value for Baud Rate register
	UBRR0H = (unsigned char)(ubrr>>8);       // based on 9600 baud rate and 16MHz clock speed.
	UBRR0L = (unsigned char)ubrr;            // assigns this vallue to the baud rate register.
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);          // enable transmit and receive
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);         // configure for 1 stop bit, with an 8 character data packet.
	UCSR0A &= ~(1<<U2X0);                    // ensure double uart speed is cleared. 
}

/***************************************************************
* This Function prompts the UART to transmit a byte.
***************************************************************/
void usart_send( uint8_t data )
{

	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
	return;
}

/***************************************************************
* This Function prompts the UART to receive a byte.
***************************************************************/
uint8_t usart_recv(void)
{

	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}
/***************************************************************
* This Function returns true when there is new data on the uart
* bus
***************************************************************/
uint8_t  usart_istheredata(void)
{
	 return (UCSR0A & (1<<RXC0));
}

void usart_display_float(float val)
{
	uint8_t i = 0, temp = 0;
	for(i= 0; i<9; i++)
	{
		temp = ((uint32_t)val)%10;
		val = val - temp;
		val = val/10;
		usart_send(temp & 0xFF);
    }

    return;
}

	



