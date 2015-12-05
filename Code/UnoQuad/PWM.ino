/*
* PWM.ino
*
* Created: 11/23/2015 3:56:21 PM
*  Author: QuocTuanIT
*/
//
#include "Servo.h"

int escPwm[5];
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void writeTo4Escs(int throttle)
{
	esc1.write(throttle);
	esc2.write(throttle);
	esc3.write(throttle);
	esc4.write(throttle);
}

void escInit()
{
	pinMode(ESC1_PIN,OUTPUT);
	pinMode(ESC2_PIN,OUTPUT);
	pinMode(ESC3_PIN,OUTPUT);
	pinMode(ESC4_PIN,OUTPUT);
	esc1.attach(ESC1_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
	esc2.attach(ESC2_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
	esc3.attach(ESC3_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
	esc4.attach(ESC4_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
	
	writeTo4Escs(0);
}

void pwmWrite(uint8_t chanel, uint16_t micro)
{
	micro=limit(micro,ESC_PWM_MIN,ESC_PWM_MAX);
	switch(chanel)
	{
		case 1: esc1.writeMicroseconds(micro); break;
		case 2: esc2.writeMicroseconds(micro); break;
		case 3: esc3.writeMicroseconds(micro); break;
		case 4: esc4.writeMicroseconds(micro); break;
	}
}