//======================================================================================
/** \file motor.h
 *    This file contains a simple motor driver.
 *
 *  Revisions:
 *    \li 04-11-2014  EB Copied "adc.h" text and changed it to accomodate motor driver functionality
 *
 *  License:
 *    This file is copyright 2014 by E Bacon, M Berry, J Fish, S Oh and released under the Lesser GNU 
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
//======================================================================================

// This define prevents this .H file from being included multiple times in a .CPP file
#ifndef _AVR_MOTOR_H_
#define _AVR_MOTOR_H_

#include "emstream.h"                       // Header for serial ports and devices
#include "FreeRTOS.h"                       // Header for the FreeRTOS RTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // Header for FreeRTOS queues
#include "semphr.h"                         // Header for FreeRTOS semaphores
#include "bit_twiddle.h"


//-------------------------------------------------------------------------------------
/** \brief This class should run a motor driver for an AVR processor. 
 *  \details The motor_driver class. contains member data that sets the various bits
 *  needed to create the motor driver. Also contains functions for setting the motor
 *  power or braking the motor.
 */

class motor
{
	protected:
		/// use this pointer to the serial port for debug info
		emstream* ptr_to_serial;
		volatile uint8_t* mc_port_md;
		volatile uint16_t* ocr_md;
		uint8_t mc_bit_a_md;
		uint8_t mc_bit_b_md;
	    
	    void init(volatile uint8_t* , uint8_t , volatile uint8_t* , uint8_t , uint8_t , uint8_t ,
			    volatile uint8_t* , volatile uint16_t* );

		int16_t pwr;
		uint16_t brk;
	

    public:
		// The constructor sets up the motor driver for use. The "= NULL" part is a
		// default parameter, meaning that if that parameter isn't given on the line
		// where this constructor is called, the compiler will just fill in "NULL".
		// In this case that has the effect of turning off diagnostic printouts
		motor (volatile uint8_t* , uint8_t , volatile uint8_t* , uint8_t , uint8_t , uint8_t ,
			    volatile uint8_t* , volatile uint16_t* , emstream* = NULL );
			    
	    motor ( uint8_t);

		// This function sets the power level of the motor object.
	    int16_t power (int16_t);
		int16_t power ( );
		
		// This function stops the motor
	    uint16_t brake (uint16_t);
		uint16_t brake ();

}; // end of class motor



// I don't know if we need to define output for the motor
//emstream& operator << (emstream&, motor_driver&);

#endif // _AVR_MOTOR_H_
