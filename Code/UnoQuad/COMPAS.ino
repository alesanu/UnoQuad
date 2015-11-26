
/*
* COMPAS.ino
*
* Created: 11/26/2015 2:01:52 PM
*  Author: QuocTuanIT
*/
/* Assign a unique ID to this sensor at the same time */

#define CUTOFF_ANGLE 5

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensors_event_t event;

void compasInit()
{
	/* Initialise the sensor */
	if(!mag.begin())
	{
		/* There was a problem detecting the HMC5883 ... check your connections */
		Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
		while(1);
	}
}

float compasGetAngle()
{
	mag.getEvent(&event);
	// Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	float heading = atan2(event.magnetic.y, event.magnetic.x);
	
	// Correct for when signs are reversed.
	if(heading < 0)
	heading += 2*PI;
	
	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
	heading -= 2*PI;
	
	// Convert radians to degrees for readability.
	float headingDegrees = heading * 180/M_PI;
	
	if (headingDegrees < CUTOFF_ANGLE || headingDegrees > (360 - CUTOFF_ANGLE) ) headingDegrees =0;
	
	return headingDegrees;
}