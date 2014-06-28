//*************************************************************************************
/** \file pid.h
 *    This file contains code for PID algorithms.
 *
 *  Revisions:
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

#include "emstream.h"                       // Header for serial ports and devices
#include "FreeRTOS.h"                       // Header for the FreeRTOS RTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // Header for FreeRTOS queues
#include "semphr.h"                         // Header for FreeRTOS semaphores


#ifndef _PID_H_
#define _PID_H_



/**
 * \brief a PID controller class
 * \details runs in a task, sets the task period, and stores state data.
 */
class pid
{
	
protected:
	/// use this pointer to the serial port for debug info
	emstream* ptr_to_serial;
	int16_t reference_position; ///This will be used with the feedback input to create the error signal
	
	float int_out_prev; ///Previous output of the integrator
	int16_t error_prev; ///Previous input to the differentiator
	
	float kprop;		/// coeff on proportional error
	float kint;			/// coeff on integral of error
	float kder;			/// coeff on derivative of error 
	
	int16_t int_clamp;	/// limits on maximum integral
	int16_t out_clamp;	/// limits on maximum pid output
	
	float s_per;		/// period of control loop - used to time task
	int8_t direction;  /// choose sign convention to match motor polarity
	
public:
	// The constructor sets up the motor driver for use. The "= NULL" part is a
	// default parameter, meaning that if that parameter isn't given on the line
	// where this constructor is called, the compiler will just fill in "NULL".
	// In this case that has the effect of turning off diagnostic printouts
	pid( float , float, float , int16_t=20, int16_t=205, float=.02, int8_t=1, emstream* = NULL );
	
	// This function sets the power level of the motor object.
	int16_t output(int16_t);
	// This function stops the motor
	int16_t reference(int16_t);
	int16_t reference();
	uint8_t period();
	void coefficients( float, float, float );
	void int_clamps( uint8_t );
	
	
};



#endif