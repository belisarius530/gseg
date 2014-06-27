/***************************************************************
* \file TWI.C
* Author: Jon Fish
* Description: This header file contains the functions needed to use
* the I2C module on the ATmega 328.
***************************************************************/

void twi_start(void);

void twi_stop(void);

void twi_write(uint8_t data);

uint8_t twi_read(uint8_t lastbit);

void twi_init(void);

void MCP28003_init(void);

void MCP28003_write(uint8_t data);
