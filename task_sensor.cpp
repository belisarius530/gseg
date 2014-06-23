//**************************************************************************************
/** \file task_sensor.cpp
 *    this file implements a task that manages data collection
 *
 *  Revisions:
 *    \li 04-17-2014 EB Copied from task_brightness
 *
 *  License:
 *    This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
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
//**************************************************************************************
#define SAMPLE_PERIOD 20
#define F_CPU 16000000
#include <util/delay.h>
#include "frt_text_queue.h"            // Header for text queue class
#include "task_sensor.h"              // Header for this task
#include "bit_twiddle.h"		//fxns to make bit operations more readable
#include "shares.h"                    // Shared inter-task communications
#include "pid.h"
#include "motor.h"

#include "adc.h"
//#include "step_test.h"
#include "imu.h"




//-------------------------------------------------------------------------------------
/** The main job of this constructor is to call the constructor of parent class
 * (\c frt_task ); the parent's constructor does the work.
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_sensor::task_sensor (const char* a_name,
								 unsigned portBASE_TYPE a_priority, 
								 size_t a_stack_size,
								 emstream* p_ser_dev
								)
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
  // Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. Each time around the for (;;)
 *  loop, the top level behavior is mediated by passing info to/from lower-level tasks
 * via shared data.
 * 
 */

frt_text_queue sensor_say(32, NULL, 10);

void task_sensor::run (void)
{
	
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
        	
	imu imu1 = imu();
	adc adc_joy_pitch = adc(1, p_serial);
	adc adc_joy_roll = adc(0, p_serial);
	DDRE &= ~(1<<PE6); //Configure input pin for control switch.
	DDRE |= (1<<PE7); // Configure output pin to disable PI/CAM.
	
	DBG( p_serial, endl<<"Task Sensor Initialization complete" );
	
	int16_t reference_pitch = 0;
	int16_t reference_roll = 0;
	uint16_t i = 0; //debug message counting timer variable
	uint8_t state = false;

	
	for (;;)
	{       state = PINE & (1<<PE6);
	        state_ctrl.put(state);
		if(!state) // If the Control Pin is LOW -> STATE: CAMERA CONTROL 
		{
		    PORTE &= ~(1<<PE7); // Disable the camera
		    imu1.get_position();
			
		    reference_pitch = adc_joy_pitch.read() - 512;
		    reference_roll = adc_joy_roll.read() - 512;
		    			
		    queue_pitch_error.put( reference_pitch/10 - imu1.pitch()*57 );
		    queue_roll_error.put( -reference_roll/15 - imu1.roll()*57 );
		}
		else
		{
		    PORTE |= (1<<PE7); // Enable the camera
		}
				
		if( i++ % 65 == 0 ){
		  //  DBG(p_serial, endl<< "Pitch:" << imu1.pitch()*57);
		    DBG(p_serial, endl<< "Roll:" << imu1.roll()*57);
		   // DBG( p_serial, endl<<"ADC0 Input: "<< reference_pitch );
		    DBG( p_serial, endl<<"ADC1 Roll Input: "<< reference_roll );
		}
	        // Sample period set to 50HZ -> 20ms
	      	delay_from_to (previousTicks, configMS_TO_TICKS (SAMPLE_PERIOD));

	}
}

