//*************************************************************************************
/** \file imu.cpp
 *    This file contains code which interfaces with the accelerometer, gyro, and 
 *    magnetometer, and fuses their data together into a predicted angle value. 
 *
 *  Revisions:
 *    \li 5-14-14 EB and JF wrote this
 *
 *
 *  License:
 *    This file is copyright 2014 by FishBaconBerryOh and released under the Lesser GNU
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************
#define F_CPU 16000000
#include <stdlib.h>                         // Include standard library header files
#include <avr/io.h>			     // Include I/O Library

#include "rs232int.h"                   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

#include <stdlib.h>                         // Include standard library header files
#include <avr/io.h>			     // Include I/O Library
#include <util/delay.h>
#include <math.h>
#include "rs232int.h"                       // Include header for serial port class
#include "bit_twiddle.h"    // Include header for serial port class
#include "pid.h"                   // Include header for the motor_driver class


//#include "gyro.h"
#include "imu.h"
extern "C" {
#include "twi.h"
}
//#include "spi.h"

//-------------------------------------------------------------------------------------
/** \brief This constructor initializes the peripherals and internal variables for the IMU.
 *  \details Run initialization functions for onboard i2c hardware, as well as the L3GD20 gyro
 *  and LSM303DLHC accelerometer/magnetometer. Additionally initialize all internal angle
 *  angle variables to zero. 
 */

imu::imu(void)
{       twi_init(); // Initialize all peripherals.
        accl_init();
	    mag_init();
	    gyro_init();
		
	    gyro_roll_angle = 0; // Initialize internal variables to zero.
        gyro_pitch_angle = 0;
	    roll_angle = 0;
        pitch_angle = 0;
        yaw_angle = 0;
        
        //gyro_x_position(void)
}


//-------------------------------------------------------------------------------------
/** \brief This function gets data from the peripherals, and calculates IMU roll and pitch.
 *  \details This function accesses data from the L3GD20 gyro and the LSM303DLHC accelerometer
 *  and magnetometer. Next it calculates roll and pitch based on that data.
 */

void imu::get_position (void)
{
    gyro_position();
    accl_position();
    mag_position();
    compute_attitude(.8);
}

//-------------------------------------------------------------------------------------
/** \brief This function
 *  \details It
 *  @return
 */

 //////////////GYRO RETURN FUNCTIONS
//-------------------------------------------------------------------------------------
/** \brief This function returns the gyro data for the x axis of rotation
 *  @return 16 bit signed integer whose value represents rotional velocity about the x axis.
 */
int16_t imu::x_gyro_val (void)
{
	return x_gyro;
}

//-------------------------------------------------------------------------------------
/** \brief This function returns the gyro data for the y axis of rotation
 *  @return 16 bit signed integer whose value represents rotional velocity about the y axis.
 */
int16_t imu::y_gyro_val (void)
{
	return y_gyro;
}

//-------------------------------------------------------------------------------------
/** \brief This function returns the gyro data for the z axis of rotation
 *  @return 16 bit signed integer whose value represents rotional velocity about the z axis.
 */
int16_t imu::z_gyro_val (void)
{
	return z_gyro;
}

//////////////ACCL RETURN FUNCTIONS
//-------------------------------------------------------------------------------------
/** \brief This function returns the accelerometer data for the x axis.
 *  @return 16 bit signed integer whose value represents the component of acceleration in 
 *  the x direction.
 */
int16_t imu::x_accl_val (void)
{
	return x_accl;
}

//-------------------------------------------------------------------------------------
/** \brief This function returns the accelerometer data for the y axis.
 *  @return 16 bit signed integer whose value represents the component of acceleration in 
 *  the y direction.
 */
int16_t imu::y_accl_val (void)
{
	return y_accl;
}

//-------------------------------------------------------------------------------------
/** \brief This function returns the accelerometer data for the z axis.
 *  @return 16 bit signed integer whose value represents the component of acceleration in 
 *  the z direction.
 */
int16_t imu::z_accl_val (void)
{
	return z_accl;
}

//////////////MAGNETOMETER RETURN FUNCTIONS
//-------------------------------------------------------------------------------------
/** \brief This function returns the magnetometer data for the x axis.
 *  @return 16 bit signed integer whose value represents magnetic field strength in 
 *  the x direction.
 */
int16_t imu::x_mag_val (void)
{
	return x_mag;
}

//-------------------------------------------------------------------------------------
/** \brief This function returns the magnetometer data for the y axis.
 *  @return 16 bit signed integer whose value represents magnetic field strength in 
 *  the y direction.
 */
int16_t imu::y_mag_val (void)
{
	return y_mag;
}

//-------------------------------------------------------------------------------------
/** \brief This function returns the magnetometer data for the z axis.
 *  @return 16 bit signed integer whose value represents magnetic field strength in 
 *  the z direction.
 */
int16_t imu::z_mag_val (void)
{
	return z_mag;
}

//-------------------------------------------------------------------------------------
/** \brief This computes the roll and pitch angles for the IMU based on sensor data.
 *  \details This function takes x,y, and z data from the accelerometer and gyro, and
 *  fuses that data together to form angle approximations using a complimentary filter.
 *  @param  alpha alpha is a weighting value which in a sense averages the gyro and 
 *  accelerometer data. This is a parameter which can be tuned iteratively.
 */
void imu::compute_attitude(float alpha)
{
    float accl_roll_angle  =  -atan2( (float)x_accl,(float)z_accl );   

    roll_angle  = alpha*(roll_angle +(float)y_gyro*.00000610865) + (1.0-alpha)*accl_roll_angle;// Collapse constants into one
                                                                                       // term, and make a #define.
                                                                                       // Make all calcs fxd pt.
    float accl_pitch_angle = atan2( (float)y_accl,(float)z_accl );

	pitch_angle = alpha*(pitch_angle+(float)x_gyro*.00000610865)+ (1.0-alpha)*accl_pitch_angle;

}         

//-------------------------------------------------------------------------------------
/** \brief This returns the calculated roll angle.
 *  @return floating point roll angle.
 */                                                                          
float imu::roll(void)
{
        return roll_angle;
}

//-------------------------------------------------------------------------------------
/** \brief This returns the calculated pitch angle.
 *  @return floating point pitch angle.
 */  
float imu::pitch(void)
{
        return pitch_angle;
}

//-------------------------------------------------------------------------------------
/** \brief Initialize the L3GD20 Gyro module, via i2c communication protocol.
 *  \details This function initializes the L3GD20 Gyro to acquire data at 95Hz, over
 *  a range of (+-)500 degrees per second.
*/
void imu::gyro_init(void)
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

//-------------------------------------------------------------------------------------
/** \brief This function acquires data from the L3GD20 Gyro module, via i2c communication.
*/
void imu::gyro_position(void)
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
	
	x_gyro = ( (uint16_t) x_h<<8)|(x_l) ; // HUGE PROBLEM HERE. DOUBLE CHECK
	y_gyro = ( (uint16_t) y_h<<8)|(y_l) ;
	z_gyro = ( (uint16_t) z_h<<8)|(z_l) ;
}

//-------------------------------------------------------------------------------------
/** \brief This function initializes the LSM303DLHC accelerometer module, via i2c communication.
*/
void imu::accl_init(void)
{
	_delay_us(1);
	twi_start();
	twi_write(0b00110010); //Write SLA + W.
	twi_write(0x20); // Write SUB address to CTRL_REG1.
	twi_write(0x47); // Set data rate to 95Hz.
	twi_stop();
}

//-------------------------------------------------------------------------------------
/** \brief This function acquires data from the LSM303DLHC accelerometer module.
*/
void imu::accl_position(void)
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
	
	x_accl = ((x_h<<8)|(x_l)) ;
	y_accl = (y_h<<8)|(y_l) ;
	z_accl = (z_h<<8)|(z_l) ;
	
}

//-------------------------------------------------------------------------------------
/** \brief This function initializes the LSM303DLHC magnetometer module, via i2c communication.
*/
void imu::mag_init(void)
{
	_delay_us(1);
	twi_start();
	twi_write(0b00111100); //Write SLA + W.
	twi_write(0x00); 
	twi_write(0x14); 
	twi_stop();
	
	_delay_us(1);
	twi_start();
	twi_write(0b00111100); //Write SLA + W.
	twi_write(0x01); 
	twi_write(0x40); 
	twi_stop();
}

//-------------------------------------------------------------------------------------
/** \brief This function acquires data from the LSM303DLHC accelerometer module.
*/
void imu::mag_position(void)
{
	uint8_t x_l = 0;
	uint8_t x_h = 0;
	uint8_t z_l = 0;
	uint8_t z_h = 0;
	uint8_t y_l = 0;
	uint8_t y_h = 0;
		
	twi_start(); // Start Condition
	twi_write(0b00111100); // SLA + W = 0x29 + w(0)
	twi_write(0x03 | 0x80); //Set Address Pointer to data register location. Setting the MSB makes sure the data autoincrements

	twi_start();
	twi_write(0b00111101); //SLA + R = 0x29 + w(1)
	
	x_l = twi_read(1);
	x_h = twi_read(1);
	
	y_l = twi_read(1);
	y_h = twi_read(1);
	
	z_l = twi_read(1);
	z_h = twi_read(0);
	
	twi_stop();

	x_mag = (x_h<<8)|(x_l) ;
	y_mag = (y_h<<8)|(y_l) ;
	z_mag = (z_h<<8)|(z_l) ;
}
