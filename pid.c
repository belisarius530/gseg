/*
 * pid.c
 *
 * Created: 6/27/2014 7:44:18 PM
 *  Author: jjfish
 */ 
#define INT_CLAMP 200
#define OUT_CLAMP 2048
#define K_INT 0.0F
#define K_PROP 1.0F
#define K_DER 0.0F

#include <avr/io.h>
#include <stdlib.h> // This library contains the abs() function.


float pid(float feedback_signal, float reference_input)
{    
	static float int_out_prev = 0; 
	static float error_prev = 0;
	
	float error;
	float der_out;
	float prop_out;
	float int_out;
		
	
	error = reference_input - feedback_signal;
	///////////////////////////////////////
	// Proportional Term Calculation
	prop_out = K_PROP*error;
	
	
	///////////////////////////////////////
	// Integral Term Calculation
	
	int_out = K_INT*(float)error + int_out_prev; 
	
	if( int_out > INT_CLAMP ) // This integrator saturation limit should be something that is specified as a parameter of the constructor
	{
		int_out = INT_CLAMP;
	}
	if( int_out < -INT_CLAMP )
	{
		int_out = -INT_CLAMP;
	}
	/*
	if(error*error_prev<0)
	{
		int_out = 0;
	}
	*/
	/*
	if( abs(error)<2 )
	{
		int_out = int_out_prev;
	}
	*/
	int_out_prev = int_out;
	//////////////////////////////////////
	// Derivative Term Calculation
	der_out = K_DER*(error - error_prev );     //derivative of measurement--constant ref drops out.
	
	error_prev = error;
	float out = prop_out + int_out + der_out;
	
	if( out  > OUT_CLAMP ) out = OUT_CLAMP;
	if( out < -OUT_CLAMP ) out = -OUT_CLAMP;
	return out;
}