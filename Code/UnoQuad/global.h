/*
 * global.h
 *
 * Created: 11/15/2015 9:53:42 AM
 *  Author: QuocTuanIT
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_

//#define OUT_RX
//#define OUT_MOTOR

#define ON		1
#define OFF		0


#define RX_THRESHOLD		50		// was 75 [10/14/2015 QuocTuanIT]
#define ARM_DELAY			500	// in ms
#define THROTTLE_OFF	5

//Min and max pulse
#define minPulseRate        1000
#define maxPulseRate        2000
#define PWM_MIN				1000
#define PWM_LOW				1145
#define PWM_MID				1300
#define PWM_MAX				2000
#define throttleChangeDelay 50

#define WAIT_MS(ms) for(unsigned long xx = millis(); (millis() - xx) < ms;)
#define WAIT_US(us) for(unsigned long xx = micros(); (micros() - xx) < us;)
#define EVERYMS(ms) static uint16_t __CONCAT(_t,__LINE__); for(uint16_t _m = millis(); _m - __CONCAT(_t,__LINE__) >= ms; __CONCAT(_t,__LINE__) = _m)

#define RX_CHANNELS		4
#define PWM_RC_LOW		1000
#define PWM_RC_MID		1504
#define PWM_RC_MAX		2100
#define THR_CUTOFF		3
#define NORMAL_CUTOFF	15

#define AIL				0
#define ELE				1
#define RUD				2
#define THR				3


#define ESC1_PIN	4
#define ESC2_PIN	5
#define ESC3_PIN	6
#define ESC4_PIN	7

typedef struct
{
	uint8_t Armed;
	uint8_t ThrottleOff;

	#define ERR_NO_ROLL			0x01
	#define ERR_NO_PITCH		0x02
	#define ERR_NO_YAW			0x04
	#define ERR_NO_THR			0x08
	#define ERR_NO_RX			(ERR_NO_ROLL | ERR_NO_PITCH | ERR_NO_YAW | ERR_NO_THR)
	uint8_t Error;
} state_t;


typedef struct
{
	unsigned int bit0 : 1;
	unsigned int bit1 : 1;
	unsigned int bit2 : 1;
	unsigned int bit3 : 1;
	unsigned int bit4 : 1;
	unsigned int bit5 : 1;
	unsigned int bit6 : 1;
	unsigned int bit7 : 1;
} volatile _bitreg8;

#define _REG_BIT2(r,b)	((*(_bitreg8*)&r).bit ## b)
#define _REG_BIT(r,b)	_REG_BIT2(r,b)

/// LED BUZZ
#define LED_PIN		12
#define BUZZ_PIN	1
#define BLINK(pin)	(digitalRead(pin)) ? (digitalWrite(pin,LOW)):(digitalWrite(pin,HIGH))

#define RX_THRESHOLD		50		// was 75 [10/14/2015 QuocTuanIT]
#define ARM_DELAY			500	// in ms
#define THROTTLE_OFF	5


int16_t limit(int16_t value, int16_t low, int16_t high)
{
	if (value < low) return low;
	else if (value > high) return high;
	else return value;
}

#endif /* GLOBAL_H_ */