/***************************************************************
* \file TWI.C
* Author: Jon Fish
* Description: This file contains the functions needed to use
* the I2C module on the ATmega 328.
***************************************************************/
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "twi.h"


/***************************************************************
* \brief This Function prompts the I2C device to perform the start condition
***************************************************************/
void twi_start(void) // 
{
	TWCR = ( (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) );

	while( (TWCR & (1<<TWINT)) == 0 )
	{
		__asm__("nop");
    }
}

/***************************************************************
* \brief This Function prompts the I2C device to perform the stop condition
***************************************************************/
void twi_stop(void)
{
	TWCR = ( (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) );
}

/***************************************************************
* \brief This Function prompts the I2C device to write one byte.
***************************************************************/
void twi_write(uint8_t data)
{
	TWDR = data;
	TWCR = ( (1<<TWINT) | (1<<TWEN) );

    while( (TWCR & (1<<TWINT)) == 0 )
	{
		__asm__("nop");
    }
}

/***************************************************************
* \brief This Function prompts the I2C device to read a byte.
* \details This Function prompts the I2C device to read a byte 
* of data from the I2C data lines in master receiver mode. If the 
* argument "lastbit" is 0, then after receiving the last byte of 
* information, the acknowledge bit is not pulled low - this indicates
* the end of the transmission. If "lastbit" is set to
* 1, the acknowledge bit will be sent, and the I2C device will 
* continue to read data. 
***************************************************************/
uint8_t twi_read(uint8_t lastbit)
{
	if(lastbit == 0)
	{
		TWCR = ( (1<<TWINT) | (1<<TWEN) );
	}
	else
	{
		TWCR = ( (1<<TWINT) | (1<<TWEA) | (1<<TWEN) );
	}

	while( ((1<<TWINT) & TWCR) == 0 )
	{
		__asm__("nop");
    }
	
    return TWDR;
}

/***************************************************************
* \brief This Function enable and initializes the I2C device.
***************************************************************/
void twi_init(void)
{
	TWBR = 0x0C; // Set transmission rate to 400kHz
	TWSR &= ~( (1<<TWPS1) | (1<<TWPS0) );
	TWCR = (1<<TWEN);
}

/***************************************************************
* \brief This Function initializes the MC28003 module.
* \details This Function initializes the MC28003 module. First, it performs
* the start condition, then performs a write operation to addresses 
* the MC28003, specifying that a write operation will be performed 
* on one of the MC23008 registers. 
* The second write operation 
* writes a value into the address pointer on the MC23008, 
* essentially specifying exactly which register will be written to. 
* (the data direction register in the case of this function). 
* Finally, 0x00 is written into the data direction register, 
* specifying that the MC28003 will be used as an output port.  
***************************************************************/
void MCP28003_init(void)
{
        _delay_us(1);
        twi_start();	       // set MCP28003 as output
	twi_write(0b01000000); //Write SLA + W
	twi_write(0x00);
	twi_write(0x00);
	twi_stop();
}

/***************************************************************
* \brief This function writes data to the output port of the MCP28003
* i/o expander.
***************************************************************/
void MCP28003_write(uint8_t data)
{
	_delay_us(1);
	twi_start();
        twi_write(0b01000000);
	twi_write(0x09);
	twi_write(data);
	twi_stop();
}
