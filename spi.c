#define F_CPU 16000000UL  
#include <avr/io.h>
#include "spi.h"
#include <util/delay.h>
#include "uart.h"

void spi_init(void)
{
	DDRB = 1<<MOSI | 1<<SCK | 1<<SS;
	   
	SPCR = (0<<SPIE) | 		//No interrupts
     (1<<SPE) | 				//SPI enabled
    (0<<DORD) | 			//shifted out LSB
    (1<<MSTR) | 			//master
    (0<<CPOL) | 			//rising leading edge   
    (0<<CPHA) | 			//sample leading edge   
    (0<<SPR1) | (0<<SPR0) ; //clock speed   
   
    SPSR = (0<<SPIF) | 		//SPI interrupt flag
    (0<<WCOL) | 			//Write collision flag
    (0<<SPI2X) ; 			//Doubles SPI clock
	
	PORTB = (1<<SS);
	return;
}


void spi_write_sseg(uint8_t data)
{
	
	SPDR = data;
	
	while( !(SPSR & (1<<SPIF)) );
	return;
}

void spi_write_dac(uint16_t data)
{
	PORTB = (0<<SS);
	//SPDR = (data>>8) & 0x1F;
	SPDR = ((data >> 8) & 0xF) | 0x70;
	while( !(SPSR & (1<<SPIF)) );
	SPDR = data & 0xFF;
	while( !(SPSR & (1<<SPIF)) );
	PORTB = (1<<SS);
	return;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//  EVERYTHING BELOW THIS LINE IS RELATED TO THE BITBANG SPI DRIVERS, EVERYTHING 
//  ABOVE IS FOR THE CLASSIC SPI DRIVERS.
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

uint8_t spi_read_gyro(uint8_t address)
{
	SPDR = 0;
	//PORTB &= ~(1<<SS);
	PORTB = (0<<SS);
	SPDR = (0x80 | address);

	while( !(SPSR & (1<<SPIF)) );
	_delay_ms(10);
	SPDR = 0xFF;
	while( !(SPSR & (1<<SPIF)) );
	PORTB = (1<<SS);
	return (SPDR);
}

uint8_t spi_xfer(uint8_t x)
{
    uint8_t value = 0;
	for(int i=7; i>=0; i--)
	{
	    PORTD &= ~(1<<clk);//make a constant named clock
		
	    if (x & (1<<i))
		{
		    PORTD |=(1<<mosi);//make constant named mosi
		}
		else
		{
		    PORTD &=~(1<<mosi);// make 
		}
		PORTD |= (1<<clk);// make miso constant
		if(PIND & (1<<miso))
		{
		    value |= (1<<i);
		}
	}
	return value;
}

void spi_write8(uint8_t address, uint8_t value)
{
    PORTD |= (1<<clk);
    PORTD &= ~(1<<cs);
    spi_xfer(address);
	spi_xfer(value);
    PORTD |= (1<<cs);
	return;
}

uint16_t spi_read16(uint8_t address)
{
    uint8_t val=0;
	uint16_t value = 0;
	PORTD |= (1<<clk);
    PORTD &= ~(1<<cs);
    spi_xfer((0x80)|(0x40)|(address & 0x3F));// repeated read command
	value = spi_xfer(0xFF);
	val = spi_xfer(0xFF);
	value |= (val<<8);
    PORTD |= (1<<cs);
	return value;
}

uint8_t spi_read8(uint8_t address)
{
    uint8_t value = 0;
	PORTD |= (1<<clk);
    PORTD &= ~(1<<cs);
	spi_xfer((0x80)|(address & 0x3F));// repeated read command
	
	value = spi_xfer(0xFF);
	PORTD |= (1<<cs);
	return value;

}

void spi_init_bitbang(void)
{
    DDRD = (0x00) | (1<<cs) | (1<<mosi) | (1<<clk);
	PORTD |= (1<<cs);
	_delay_ms(10);
	spi_write8(L3GD20_REGISTER_CTRL_REG1, 0x0F);
	return;
}
