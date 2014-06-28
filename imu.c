/*
 * imu.c
 *
 * Created: 6/26/2014 7:50:35 PM
 *  Author: jjfish
 */ 
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <math.h>
#include "twi.h"
#include "imu.h"


/*
void get_position (void)
{
	gyro_position();
	accl_position();
	compute_attitude(.8);
}
*/

float compute_pitch(float alpha, int16_t y_accl, int16_t z_accl, int16_t x_gyro)
{
	static float pitch_angle = 0.0;
	
	//float accl_pitch_angle = atan2( (float)(y_accl),(float)(z_accl) );
    float accl_pitch_angle = atan2( (float)(z_accl),(float)(-y_accl) );
	
	pitch_angle = alpha*(pitch_angle+(float)(x_gyro)*.00000610865)+ (1.0-alpha)*accl_pitch_angle;

    return pitch_angle;

}

void gyro_init(void)
{
	_delay_us(1);
	twi_start();
	twi_write(0b11010110); //Write SLA + W.
	twi_write(0x20); // Write SUB address to CTRL_REG1.
	twi_write(0x0F); // Set data rate to 95Hz.
	twi_stop();

	_delay_ms(1); //_delay_us(1);
	
	twi_start();
	twi_write(0b11010110); //Write SLA + W.
	twi_write(0x23); // Write SUB address to CTRL_REG4.
	twi_write(0x10); // Set range to 500degrees/second
	twi_stop();
}

void gyro_position(int16_t *x_gyro, int16_t *y_gyro, int16_t *z_gyro)
{
	uint8_t x_l = 0;
	uint8_t x_h = 0;
	uint8_t y_l = 0;
	uint8_t y_h = 0;
	uint8_t z_l = 0;
	uint8_t z_h = 0;
	
	twi_start(); // Start Condition
	twi_write(0b11010110); // SLA + W = 0x29 + w(0)
	twi_write(0x28 | 0x80); //Set Address Pointer to data register location. Setting the MSB makes sure the data autoincrements
	
	twi_start();
	twi_write(0b11010111); //SLA + R = 0x29 + w(1)
	
	x_l = twi_read(1);
	x_h = twi_read(1);
	y_l = twi_read(1);
	y_h = twi_read(1);
	z_l = twi_read(1);
	z_h = twi_read(0);
	
	twi_stop();
	
	*x_gyro = ( (uint16_t) x_h<<8)|(x_l) ; 
	*y_gyro = ( (uint16_t) y_h<<8)|(y_l) ;
	*z_gyro = ( (uint16_t) z_h<<8)|(z_l) ;
}

void accl_init(void)
{
	_delay_us(1);
	twi_start();
	twi_write(0b00110010); //Write SLA + W.
	twi_write(0x20); // Write SUB address to CTRL_REG1.
	twi_write(0x47); // Set data rate to 95Hz.
	twi_stop();
}

void accl_position(int16_t *x_accl, int16_t *y_accl, int16_t *z_accl)
{
	uint8_t x_l = 0;
	uint8_t x_h = 0;
	uint8_t z_l = 0;
	uint8_t z_h = 0;
	uint8_t y_l = 0;
	uint8_t y_h = 0;
	
	twi_start(); // Start Condition
	twi_write(0b00110010); // SLA + W = 0x29 + w(0)
	twi_write(0x28 | 0x80); //Set Address Pointer to data register location. Setting the MSB makes sure the data autoincrements
	
	twi_start();
	twi_write(0b00110011); //SLA + R = 0x29 + w(1)
	
	x_l = twi_read(1);
	x_h = twi_read(1);
	y_l = twi_read(1);
	y_h = twi_read(1);
	z_l = twi_read(1);
	z_h = twi_read(0);
	twi_stop();
	
	*x_accl = ((x_h<<8)|(x_l)) ;
	*y_accl = (y_h<<8)|(y_l) ;
	*z_accl = (z_h<<8)|(z_l) ;
	
}