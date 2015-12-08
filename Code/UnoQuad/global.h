/*
 * global.h
 *
 * Created: 11/15/2015 9:53:42 AM
 *  Author: QuocTuanIT
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_

//#define DEBUG
//#define OUT_BAT
//#define OUT_MOTOR
//#define OUT_RX
//#define OUT_GYRO
//#define USE_CMD

#define ESC1_PIN	4
#define ESC2_PIN	5
#define ESC3_PIN	6
#define ESC4_PIN	7

#define RX_AIL_PIN	8
#define RX_ELE_PIN	9
#define RX_THR_PIN	10
#define RX_RUD_PIN	11
#define RX_AUX_PIN	12

#define ON		1
#define OFF		0

#define LOW_VOLTAGE		113		//alarm Low voltage 11.3V

#define ARM_DELAY			500	// in ms
#define RX_THRESHOLD		50  // arming ON/OFF
#define THROTTLE_OFF		5


#define ESC_PWM_MIN			1000
#define ESC_PWM_MAX			2000

//pulse MAX/MIN for RX PWM
#define PWM_RC_LOW		1000
#define PWM_RC_MID		1504
#define PWM_RC_MAX		2100
#define THR_CUTOFF		3
#define NORMAL_CUTOFF	15

#define AIL				0
#define ELE				1
#define RUD				2
#define THR				3
#define AUX				4

#define ROL 0
#define PIT 1
#define YAW 2

typedef struct {
	struct pidGain {float Kp,Ki,Kd; int max;};
	struct pidGain roll;
	struct pidGain pitch;
	struct pidGain yaw;
} pid_param_t;

typedef struct{
	struct pidstate {float iTerm, lastDErr;};
	struct pidstate	roll;
	struct pidstate pitch;
	struct pidstate yaw;
} pid_state_t;

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
#define LED_PIN		13
#define BUZZ_PIN	15
#define BLINK(pin)	(digitalRead(pin)) ? (digitalWrite(pin,LOW)):(digitalWrite(pin,HIGH))

void beep(uint16_t t)
{
  digitalWrite(BUZZ_PIN,HIGH);
  for(uint16_t i=0; i<t; i++ ) delay(1);
  digitalWrite(BUZZ_PIN,LOW);
}


double limit(double value, double low, double high)
{
	if (value < low) return low;
	else if (value > high) return high;
	else return value;
}

#define WAIT_MS(ms) for(unsigned long xx = millis(); (millis() - xx) < ms;)
#define WAIT_US(us) for(unsigned long xx = micros(); (micros() - xx) < us;)
#define EVERYMS(ms) static uint16_t __CONCAT(_t,__LINE__); for(uint16_t _m = millis(); _m - __CONCAT(_t,__LINE__) >= ms; __CONCAT(_t,__LINE__) = _m)

#endif /* GLOBAL_H_ */
