//**************************************************************************************
/** \file task_motor.cpp
 *    this file implements a task that manages top level system behavior
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
#define SAMPLE_PERIOD 50
#define F_CPU 16000000
#include <util/delay.h>
#include "frt_text_queue.h"            // Header for text queue class
#include "task_motor.h"              // Header for this task
#include "bit_twiddle.h"		//fxns to make bit operations more readable
#include "shares.h"                    // Shared inter-task communications
#include "pid.h"
#include "motor.h"
#include "adc.h"
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

task_motor::task_motor (const char* a_name,
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

frt_text_queue motor_say(32, NULL, 10);

void task_motor::run (void)
{
	
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
        	
	pid pid_pitch_imu = pid(8.0, .3, 40 , 300, 350, .02, -1); ///pid controller
	pid pid_roll_imu = pid(3.0, .5, 20.0 , 150, 150, .02, 1); ///pid controller
	
	pid pid_pitch_picam = pid(8.0, .1, 40 , 200, 300, .05, -1); ///pid controller
	pid pid_roll_picam = pid(4.0, .1, 20.0 , 150, 200, .05, 1); ///pid controller
	
	motor motor_pitch = motor(2); 			///motor manager
	motor_pitch.power(0 );
	//adc adc1 = adc(1, p_serial ); 		///adc reader
	motor motor_roll = motor(1); 			/// another motor, one matched to encoder 2
	motor_roll.power(0 );					///initial power 0

	pid_roll_imu.reference(0);
	pid_pitch_imu.reference(0);	
	pid_roll_picam.reference(0);
	pid_pitch_picam.reference(0);
	int16_t roll_error = 0;
	int16_t pitch_error = 0;
	uint16_t i = 0;
	
	DBG( p_serial, endl<<"Task Motor Initialization complete" );
	for (;;)
	{       
	        i++;
		
	        roll_error = queue_roll_error.get();
		pitch_error = queue_pitch_error.get();
	        if( state_ctrl.get() )
		{
		    motor_roll.power( pid_roll_picam.output( roll_error ) );
		    motor_pitch.power( pid_pitch_picam.output( pitch_error ) );
		    if( !(i%65) )
		    {
		        //DBG( p_serial, endl<<"State:1, Pitch Motor Power:"<<motor_roll.power() );
		    }
		    
		}
		else 
		{
		    motor_roll.power( -pid_roll_imu.output( roll_error ) );
		    motor_pitch.power( pid_pitch_imu.output( pitch_error ) );
		    if( !(i%65) )
		    {
		        //DBG( p_serial, endl<<"State:0, Pitch Motor Power:"<<motor_roll.power() );
		        DBG( p_serial, endl<<"roll_error: "<<roll_error );
			DBG( p_serial, endl<<"roll_power: "<<motor_roll.power() );
		    }
		}
	        // Sample period set to 50HZ -> 20ms
		delay_from_to (previousTicks, configMS_TO_TICKS (1));

	}
}

