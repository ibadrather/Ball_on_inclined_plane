/*
 * PID.c
 *
 * Created: 10.08.1921 12:40:11
 *  Author: Henry Ford
 */ 
// Compiler includes
#include <math.h>
#include <avr/io.h>
// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
// Modul includes
#include "global.h"
#include "PID.h"
//----------------------------------------------------------
void  vPID ( void * pvParameters)
{  float x = 0.5, y = 1.0 , st = 0.015, error , lasterror = 0.0 , errortotal = 0.0 ;
	// Place here your local parameters
	// st = float(0.018) // seconds
  // Super loop of task
  for (;;)  
  { // Get position value from sensor queue
    xQueueReceive (QueueSensor, &x, portMAX_DELAY);
		
		
		/*	Compute here your digital PID formula with the 
				help of following global variables:
				reference, proportional, integral, derivative
				and please remember the sampling time
		*/
		error = (reference -x);
		
		errortotal  =  errortotal + error  ;
		
		
		y = ( proportional * error ) + integral*(errortotal * st ) + derivative * ((error - lasterror) / st);
		
		lasterror = error ;
		
		
		// For example a simple P control startup formula  
		// y = proportional * (reference - x);
		
		
		// Write angle value to servo queue
		xQueueSend (QueueServo, &y, portMAX_DELAY);
		
	} // end for loop
} // end of task PID
//----------------------------------------------------------
