/*
* UnoQuad.ino
*
* Created: 11/15/2015 9:50:07 AM
*  Author: QuocTuanIT
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <avr/wdt.h>
#include "SerialCommand.h"
#include "global.h"

int battery_voltage;

double gyroRaw[3];
float  gyroRate[3];
double gyroCal[3];
float  setPoint[3];
float  pidOut[4];// ROLL > PITCH > YAW > COMPAS

pid_param_t pid;
pid_state_t pidState;
state_t State;
SerialCommand command;

extern int escPwm[5];
extern int RX[4];
extern int RX_raw[4];
extern uint8_t RX_good;

static void checkState()
{
	State.ThrottleOff = RX[THR] < THROTTLE_OFF;
	uint8_t e = 0;
	e |= (~RX_good) & (ERR_NO_PITCH|ERR_NO_ROLL|ERR_NO_THR|ERR_NO_YAW);
	
	State.Error = e;
}

static void arm(uint8_t value)
{
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

int batVoltage()
{
	static int batt;
	//A complementary filter is used to reduce noise.
	//0.09853 = 0.08 * 1.2317.
	batt = batt * 0.92 + (analogRead(0) + 65) * 0.09853;
	return batt;
}
void mixers(int throttle)
{
	//	  (4)\   /(1)
	//        \ /              x
	//         X		       |
	//        / \         y____|
	//    (3)/   \(2)
	//
	//region of mixer PID Roll-Pitch-Yaw-Throttle
	if (throttle > 1800) throttle = 1800; //full throttle.
	escPwm[1] = throttle - pidOut[PIT] + pidOut[ROL] - pidOut[YAW];
	escPwm[2] = throttle + pidOut[PIT] + pidOut[ROL] + pidOut[YAW];
	escPwm[3] = throttle + pidOut[PIT] - pidOut[ROL] - pidOut[YAW];
	escPwm[4] = throttle - pidOut[PIT] - pidOut[ROL] + pidOut[YAW];

	//region of battery low voltage compensate
	int bat = batVoltage();
	if (bat < 1240 && bat > 800)
	{
		for (uint8_t i=1; i<=4; i++) escPwm[i] += escPwm[i] * ((1240 - bat)/(float)3500);
	}
	//region of limit output
	for (uint8_t i=1; i<=4; i++)
	{
		escPwm[i] = limit(escPwm[i],1200,2000);
	}
}

void setup()
{
	MCUSR = 0;
	wdt_disable(); //disable watchdog timer
	pinMode(LED_PIN,OUTPUT);
	Serial.begin(115200);
	pwmInit();
	pidInit();
	#ifdef USE_CMD
	commandInit();
	#endif
	delay(2000);
	compasInit();
	gyroInit();
	delay(250); //Gyro Stable

	gyroCalibration();
	
	rxInit();
	
	digitalWrite(LED_PIN,HIGH);
	Serial.println("Init Success");
}

float compasAngle;
float compasPID;
float compasSP = 290;
bool compasFlag;
#define CW	1
#define CCW	0

void loop()
{
	gyroReadRaw();
	gyroCaculate();
	rxRead();
	checkState();
	armingLoop();
	#ifdef USE_CMD
	command.readSerial();
	#endif

	calculate_pid();
	
	compasAngle = compasGetAngle();
	if (compasAngle > (215 + compasSP ) || compasAngle < compasSP)
	{
		compasFlag = CW;
		compasPID = 360 - compasAngle + compasSP;
	}
	else if ( compasAngle > compasSP && compasAngle < (compasSP + 215))
	{
		compasFlag = CCW;
		compasPID = compasAngle;
	}
	caculate_pid_compas(compasPID , compasSP);
	
	if (State.Armed && !State.ThrottleOff)
	{
		mixers(RX_raw[THR]);

		if(RX[AIL] == 0) setPoint[ROL] = -0.28;
		else  setPoint[ROL] = RX[AIL]/3;
		
		//	setPoint[ROL] = RX[AIL]/3;
		setPoint[PIT] = -RX[ELE]/3;
		
		if (RX[RUD] == 0)
		{
			if ( compasFlag = CW)
			{
				setPoint[YAW] = pidOut[COM];
			}
			else setPoint[YAW] = -pidOut[COM];
		}
		else setPoint[YAW] = RX[RUD]/3;
	}
	else
	{
		setPoint[ROL] = 0;
		setPoint[PIT] = 0;
		setPoint[YAW] = 0;
		escPwm[1] = escPwm[2] = escPwm[3] = escPwm[4] = 1000;
	}

	pwmOutput();
	
	
	#ifdef OUT_PID
	Serial.print("COMPAS: "); Serial.print(compasAngle); Serial.print("\t");
	Serial.println(pidOut[COM]);
	#endif
	
	#ifdef OUT_RX
	Serial.print("AIL: "); Serial.print(RX[AIL]); Serial.print("\t");
	Serial.print("ELE: "); Serial.print(RX[ELE]); Serial.print("\t");
	Serial.print("RUD: "); Serial.print(RX[RUD]); Serial.print("\t");
	Serial.print("THR: "); Serial.print(RX_raw[THR]); Serial.print("\n");
	delay(100);
	#endif
	
	#ifdef OUT_GYRO
	Serial.print("R: "); Serial.print(gyroRate[ROL]); Serial.print("   ");
	Serial.print("P: "); Serial.print(gyroRate[PIT]); Serial.print("   ");
	Serial.print("Y: "); Serial.print(gyroRate[YAW]); Serial.print("\n");
	delay(100);
	#endif
	
	#ifdef OUT_COMPAS
	Serial.print("COMPAS: "); Serial.print(compasGetAngle()); Serial.print("\n");
	#endif
}