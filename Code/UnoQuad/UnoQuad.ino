/*
* UnoQuad.ino
*
* Created: 11/15/2015 9:50:07 AM
*  Author: QuocTuanIT
*/
#include <Wire.h>
#include <avr/wdt.h>
#include "SerialCommand.h"
#include "global.h"

float batVol;

float  setPoint[3];
float  pidOut[3];

pid_param_t pid;
pid_state_t pidState;
state_t State;
SerialCommand command;

extern int escPwm[5];
extern int RX[5];
extern int RX_raw[5];
extern int RX_isr[5];
extern uint8_t RX_good;
extern double gyroRaw[3];
extern double gyroRate[3];
extern double gyroCal[3];

static void checkState()
{
	State.ThrottleOff = RX[THR] < THROTTLE_OFF;
	uint8_t e = 0;
	e |= (~RX_good) & (ERR_NO_PITCH|ERR_NO_ROLL|ERR_NO_THR|ERR_NO_YAW);
	
	State.Error = e;
}

static void arm(uint8_t value)
{
	beep(100);
	if (value && !State.Armed)
	{
		State.Armed = ON;
		digitalWrite(LED_PIN,LOW);
		pidReset();
	}
	else if (!value && State.Armed)
	{
		State.Armed = OFF;
		digitalWrite(LED_PIN,HIGH);
	}
}

static void armingLoop()
{
	static uint16_t startArm;
	static uint16_t startOff;
	uint16_t t = millis();
	
	if (State.Error != 0) return;
	
	if (startArm == 0)
	{
		if (State.ThrottleOff && abs(RX[RUD]) > RX_THRESHOLD)
		startArm = t;
	}
	else if (!State.ThrottleOff || abs(RX[RUD]) < RX_THRESHOLD) startArm = 0;
	else if (t - startArm >= ARM_DELAY)
	{
		if (RX[RUD] > RX_THRESHOLD) arm(ON);
		else arm(OFF);
		startArm = 0;
	}
}

void batVoltage()
{
	batVol =(analogRead(0) + 25) * 0.48828f;
	if (batVol > 60 && batVol < LOW_VOLTAGE) digitalWrite(BUZZ_PIN,HIGH);
	else digitalWrite(BUZZ_PIN,LOW);
}
void mixers(int throttle)
{
	//	  (1)\   /(2)
	//        \ /				x
	//         X				|
	//        / \          y____|
	//    (4)/   \(3)
	//
	//region of mixer PID Roll-Pitch-Yaw-Throttle
	if (throttle > 1800) throttle = 1800; //full throttle.
	escPwm[1] = throttle - pidOut[ROL] - pidOut[PIT] + pidOut[YAW];
	escPwm[2] = throttle + pidOut[ROL] - pidOut[PIT] - pidOut[YAW];
	escPwm[3] = throttle + pidOut[ROL] + pidOut[PIT] + pidOut[YAW];
	escPwm[4] = throttle - pidOut[ROL] + pidOut[PIT] - pidOut[YAW];


	//region of battery low voltage compensate
	if (batVol > 60 && batVol < 125)
	{
		for (uint8_t i=1; i<=4; i++) escPwm[i] += escPwm[i] * ((125 - batVol)/350);
	}
	//region of limit output
	for (uint8_t i=1; i<=4; i++)
	{
		escPwm[i] = limit(escPwm[i],ESC_PWM_MIN,ESC_PWM_MAX);
	}
}

void setup()
{
	MCUSR = 0;
	wdt_disable(); //disable watchdog timer
	pinMode(LED_PIN,OUTPUT); pinMode(BUZZ_PIN,OUTPUT); digitalWrite(BUZZ_PIN,LOW);
	Serial.begin(115200);
	escInit();
	pidInit();
	gyroInit();
	delay(250);
	gyroCalibration();
	rxInit();
	#ifdef USE_CMD
	commandInit();
	#endif
	digitalWrite(LED_PIN,HIGH);
	beep(500);
	Serial.println("Init Success");
}

int throttleCapture;
int landing()
{
	EVERYMS(1000) throttleCapture = throttleCapture*0.99;
	if (throttleCapture<900)
	{
		throttleCapture = 900;
		arm(OFF);
	}
	
	return throttleCapture;
}

void loop()
{
	gyroReadRaw();
	gyroCaculate();
	batVoltage();
	rxRead();
	checkState();
	armingLoop();

	EVERYMS(4) calculate_pid();
	
	if (State.Armed && !State.ThrottleOff)
	{
		mixers(RX_raw[THR]);
		
		if (RX[AUX] > 0)
		{
			setPoint[ROL] = -RX[AIL]/2.5;
			setPoint[PIT] =  RX[ELE]/2.5;
			setPoint[YAW] = -RX[RUD]/2.5;
		}
		else
		{
			setPoint[ROL] =  RX[AIL]/2.5;
			setPoint[PIT] = -RX[ELE]/2.5;
			setPoint[YAW] =  RX[RUD]/2.5;
		}
	}
	else
	{
		setPoint[ROL] = 0;
		setPoint[PIT] = 0;
		setPoint[YAW] = 0;
		escPwm[1] = escPwm[2] = escPwm[3] = escPwm[4] = 1000;
	}
	
	for(uint8_t i=1; i<=4; i++) pwmWrite(i,escPwm[i]);
	
	#ifdef USE_CMD
	command.readSerial();
	#endif
	#ifdef DEBUG
	debugProcess();
	#endif
}
