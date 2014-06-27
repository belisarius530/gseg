//*************************************************************************************
/** \file pid.cpp
 * 
 *
 *  Revisions:
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
 *    CAUSED AND ON ANY THEORY OF LIABILI = 0TY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

#include <stdlib.h>                         // Include standard library header files
#include <avr/io.h>			     // Include I/O Library

#include "rs232int.h"
//*************************************************************************************

#include <stdlib.h>                         // Include standard library header files
#include <avr/io.h>			     // Include I/O Library

#include "rs232int.h"                       // Include header for serial port class
#include "bit_twiddle.h"    // Include header for serial port class
#include "pid.h"                   // Include header for the motor_driver class

//-------------------------------------------------------------------------------------
/** \brief This constructor sets up a PID controller
 *  \details This object sets up and runs a PID controller with the given parameters.
 * Uses the backward difference transform to implement digital coeffs to implement digital
 * version of the continuous time equivalent PID system. The system is run by calling 
 * \c output( feedback_signal ). The PID object internally calculates error signal and stores state.
 * It operates on the calculated error signal to provide output.
 *
 *  @param kprop_in A pointer to the serial port which writes debugging info.
 *  @param kint_in A pointer to the PWM data direction register
 *  @param kder_in A pointer to the PWM settings bit
 *  @param iClamp A pointer to the motor controller data direction register
 *  @param oClamp The first motor controller settings bit
 *  @param tm The second motor controller settings bit
 *  @param dir sign convention for correcting motor- depends on motor polarity
 *  @param serpt ptr to output buffer
 */

pid::pid(float kprop_in , float kder_in ,float kint_in, int16_t iClamp, int16_t oClamp, float tm, int8_t dir, emstream* serpt)
{
	s_per = tm;
	
	kprop = kprop_in;
	kint = kint_in * s_per;
	kder = kder_in / s_per;
	
	reference_position = 0;
	ptr_to_serial = serpt;
	
	int_clamp = iClamp;
	out_clamp = oClamp;

	direction = dir;
	
	
}


//----------------------->--------------------------------------------------------------
/** \brief Calculate the output of PID control based on current input and state.
 *  \details given the \c feedback signal, calculate the error and the P, I and D terms,
 * limit the integrator term and output to the max levels, update I and D state, and return 
 * the motor control signal
 *  @param feedback_signal the position feedback from an optical encoder
 *  @return pid output for motor
 */
int16_t pid::output (int16_t feedback_signal)
{
	int16_t error;
	float der_out;
	float prop_out;
	float int_out;
	int16_t pid_output;
	
	error = reference_position - feedback_signal;
	///////////////////////////////////////
	// Proportional Term Calculation
	prop_out = kprop*error;
	
	
	///////////////////////////////////////
	// Integral Term Calculation
	
	int_out = kint*(float)error + int_out_prev;
	
	if( int_out > int_clamp ) // This integrator saturation limit should be something that is specified as a parameter of the constructor
        {
		int_out = int_clamp;
	}
	if( int_out < -int_clamp )
	{
		int_out = -int_clamp;
	}
	
	if(error*error_prev<0)
	{
		int_out = 0;
	}
		
	if( abs(error)<2 )
	{
	    int_out = int_out_prev;
	}
	int_out_prev = int_out;
	//////////////////////////////////////
	// Derivative Term Calculation
	der_out = kder*(error - error_prev );     //derivative of measurement--constant ref drops out.
        
	error_prev = error;
	int16_t out = prop_out + int_out + der_out;
	
	if( out  > out_clamp ) out = out_clamp;
	if( out < -out_clamp ) out = -out_clamp;
	return out*direction;
}


//-------------------------------------------------------------------------------------
/** \brief Set the reference position
 *  \details Change the commanded position that the PID moves the motor to
 *  @param reference_position_input the new desired motor encoder position
 *  @return reference position
 */
int16_t pid::reference (int16_t reference_position_input)
{
	reference_position = reference_position_input;
}


/** \brief Get the reference position
 *  \details Read the commanded position that the PID moves the motor to
 *  @return reference position
 */
int16_t pid::reference ()
{
	return reference_position;
}


/** \brief Get the pid period
 *  \details Read the sample period of the pid.
 *  @return sample period
 */
uint8_t pid::period()
{
	return s_per*1000;
}


/**
 * \brief Set the coefficients for the PID
 * \details "Hot-change" the PID tuning
 * @param kp float of prop gain
 * @param ki float integral gain
 * @param kd float deriv gain
 */
void pid::coefficients( float kp, float ki, float kd)
{
	kprop = kp;
	kint = ki * s_per;
	kder = kd / s_per;
}

/**
 * \brief Change the integral clamp value
 * \details set the max integrator output
 * @param clamp the new clamp value
 */
void pid::int_clamps( uint8_t clamp )
{
	int_clamp = clamp;

}