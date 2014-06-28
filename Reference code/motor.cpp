//*************************************************************************************
/** \file motor.cpp
 *    This file contains a very simple motor driver. 
 *
 *  Revisions:
 *    \li 4-14-14 EB and JF wrote this
 *    \li 4-16-14 JF added saturation limits to the power params
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

#include <stdlib.h>                         // Include standard library header files
#include <avr/io.h>			     // Include I/O Library

#include "rs232int.h"                       // Include header for serial port class
#include "motor.h"                   // Include header for the motor_driver class
#include "bit_twiddle.h"

//-------------------------------------------------------------------------------------
/** \brief This constructor sets up an motor
 *  \details motor function drives the PWM to the motor. The function \c set_power()
 * 	tells the motor how much torque to auint8_tpply. Sets the polarity. The function
 * 	\c brake() will apply damping to the motor to stop the motor.
 *  
 *  @param p_serial_port A pointer to the serial port which writes debugging info.
 *  @param pwm_ddr A pointer to the PWM data direction register
 *  @param pwm_bit A pointer to the PWM settings bit 
 *  @param mc_ddr A pointer to the motor controller data direction register
 *  @param mc_bit_a The first motor controller settings bit
 *  @param mc_bit_b The second motor controller settings bit 
 *  @param mc_bit_diag The motor controller output enable and diagnostic register
 *  @param mc_port The motor controller port
 *  @param ocr The compare register for setting the PWM duty cycle
 *  @param p_serial_port The debug output port
 */
motor::motor(volatile uint8_t* pwm_ddr,     uint8_t pwm_bit,
			 volatile uint8_t* mc_ddr,      uint8_t mc_bit_a,
			 uint8_t mc_bit_b,              uint8_t mc_bit_diag,
			 volatile uint8_t* mc_port,     volatile uint16_t* ocr,
			 emstream* p_serial_port)
{
	
	//initializion procedure unique to each motor
	init( pwm_ddr, pwm_bit, mc_ddr, mc_bit_a, mc_bit_b, mc_bit_diag, mc_port, ocr );
	ptr_to_serial = p_serial_port;
	
}


/** \brief This constructor sets up a motor using "standard settings"
 *  \details motor(1) will set up standard motor controller 1, and motor(2) will
 *  do likewise. That is all teh standard motors. if you send any number besides
 *  1 or 2, b/c i dont want to figure out exceptions for that.
 *  @param num 1 or 2 (later, maybe, 3 too ;) which standard motor to set up
 */
motor::motor( uint8_t num )
{
	//select what standard motor configuration to do
	if( num == 2 )
	{
		//motor 2 setup
		init( &DDRB, (uint8_t)PB5, &DDRD, (uint8_t)PD5, (uint8_t)PD6, (uint8_t)PD7, &PORTD, &OCR1A );
	}
	else
	{
		//motor 1 setup
		init( &DDRB, (uint8_t)PB6, &DDRC, (uint8_t)PC0, (uint8_t)PC1, (uint8_t)PC2, &PORTC, &OCR1B );
	}
	
}


/**
 * \brief initialize motor settings common to all motor setups
 * \details runs no matter which standard motor setup (1 or 2) you are making
 *  @param pwm_ddr A pointer to the PWM data direction register
 *  @param pwm_bit A pointer to the PWM settings bit 
 *  @param mc_ddr A pointer to the motor controller data direction register
 *  @param mc_bit_a The first motor controller settings bit
 *  @param mc_bit_b The second motor controller settings bit 
 *  @param mc_bit_diag The motor controller output enable and diagnostic register
 *  @param mc_port The motor controller port
 *  @param ocr The compare register for setting the PWM duty cycle
 * 
 */
void motor::init(  volatile uint8_t* pwm_ddr,       uint8_t pwm_bit,
				   volatile uint8_t* mc_ddr,        uint8_t mc_bit_a, 
				   uint8_t mc_bit_b,                uint8_t mc_bit_diag,
				   volatile uint8_t* mc_port,       volatile uint16_t* ocr
)
{
	
	/**
	 * INITIALIZATION STEPS UNIQUE TO EACH MOTOR INSTANTIATED
	 */
	
	// Save important member data, including port register addresses and bits.
	mc_port_md = mc_port; 
	ocr_md = ocr;
	mc_bit_a_md = mc_bit_a;
	mc_bit_b_md = mc_bit_b;
	
	//Set motor 1 PWM pin as an output.
	set_bit(*pwm_ddr, pwm_bit, 1);
	
	// Set motor 1 control pins as outputs.
	set_bit(*mc_ddr, mc_bit_a, 1);
	set_bit(*mc_ddr, mc_bit_b, 1);
	
	// Enable the internal pullup resistor for the Diagnostic Enable
	set_bit(*mc_ddr, mc_bit_diag, 0);
	
	// pin of the motor controller.
	set_bit(*mc_port, mc_bit_diag, 1);
	
	
	/**
	 * INITIALIZATION COMMON TO ALL MOTORS
	 */
	
	// Set 10-bit Fast PWM mode, and configure
	set_bit(TCCR1A, WGM11, 1);
	set_bit(TCCR1A, WGM10, 1);
	set_bit(TCCR1A, COM1A1, 1);
	set_bit(TCCR1A, COM1B1, 1);
	
	
	// the PWM pin to set at BOTTOM, and
	// clear at a compare match.
	set_bit(TCCR1A, COM1A0, 0);
	set_bit(TCCR1A, COM1B0, 0);
	
	// Set Clock prescaler to 1/8
	TCCR1B = 0;
	set_bit(TCCR1B, WGM12, 1);
	set_bit(TCCR1B, CS11, 1);
	
	//start out freewheelin
	power(0);
	
}                                        

//-------------------------------------------------------------------------------------
/** \brief This function sets the power level for the motor
 *  \details positive input drives positive torque and negative drives negative torque
 *  @param  power power level from -1023 to +1023. negative and positive indicate direction of motor spin
 *  @return power level
 */
int16_t motor::power (int16_t p)
{
	
	brk = 0; //zero the brake
	
	/**
	 * set motor direction by polarity of sign (+/-) power
	 */
	if(p >= 0)
	{
		
		set_bit(*mc_port_md, mc_bit_a_md, 1);
		//*mc_port_md |= (1<<mc_bit_a_md); //set to 10, or 01 or whatever is necessary use member data
		set_bit(*mc_port_md, mc_bit_b_md, 0); 
		//*mc_port_md &= ~(1<<mc_bit_b_md); // setting things on one of the ports
	}
	else
	{
		set_bit(*mc_port_md, mc_bit_a_md, 0);
		set_bit(*mc_port_md, mc_bit_b_md, 1); 
		//*mc_port_md |= (1<<mc_bit_b_md); //set to 10, or 01 or whatever is necessary use member data
		//*mc_port_md &= ~(1<<mc_bit_a_md); // setting things on one of the ports
	}
	
	
	/**
	 * limit power output to max of pwm
	 */
	if(p>1023)
	{
		p = 1023;// This imposes a saturation based limiting of the motor power.
	}
	if(p<-1023)
	{
		p = -1023;
	}
	pwr = p;
	
	
	/**
	 * the commanded pwm duty cycle is unsigned, so strip sign now
	 */
	*ocr_md = abs(p); //set the PWM compare register to the given power setting
	
	return pwr;
}


/**
 * \brief read current motor power
 */
int16_t motor::power ()
{
	return pwr;
}


/** \brief This function applies damping to the motor to brake
 *  \details It creates a short between the two leads of the motor. Can vary the strength of damping
 *  @param b The power of the damping applied to the brake. 0 is freewheeling, 1023 is max damping
 *  @return none
 */
uint16_t motor::brake (uint16_t b)
{
	pwr = 0; //zero power
	
	
	/**
	 * h bridge setting for damping-- short both sides of motor
	 */
	set_bit(*mc_port_md, mc_bit_a_md, 0);
	set_bit(*mc_port_md, mc_bit_b_md, 0);	
	
	
	/**
	 * limit commanded damping power to within limits of pwm
	 */
	if(b>1023)
	{
		b = 1023;// This imposes a saturation based limiting of the brake power.
	}
	brk = b;
	// setting things on one of the ports
	*ocr_md = b;
	
	return brk;
}


/**
 * \brief read the current brake setting
 */
uint16_t motor::brake()
{
	return brk;
}

