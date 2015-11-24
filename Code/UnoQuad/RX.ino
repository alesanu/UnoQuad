
/*
* RX.ino
*
* Created: 11/24/2015 1:09:35 PM
*  Author: QuocTuanIT
*/
#include "PinChangeInt.h"

int RX[4];
int RX_raw[4];
uint8_t RX_good;
int RX_isr[4];
static uint8_t _mode;
static uint8_t _RX_good;

#define RX_AIL_PIN 8
#define RX_ELE_PIN 9
#define RX_THR_PIN 10
#define RX_RUD_PIN 11
#define EVERYMS(ms) static uint16_t __CONCAT(_t,__LINE__); for(uint16_t _m = millis(); _m - __CONCAT(_t,__LINE__) >= ms; __CONCAT(_t,__LINE__) = _m)

void rxInit()
{
	PCintPort::attachInterrupt(RX_THR_PIN, thr_callback,CHANGE);
	PCintPort::attachInterrupt(RX_RUD_PIN, rud_callback,CHANGE);
	PCintPort::attachInterrupt(RX_AIL_PIN, ail_callback,CHANGE);
	PCintPort::attachInterrupt(RX_ELE_PIN, ele_callback,CHANGE);
}

void thr_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_THR_PIN))
	_start = t;
	else
	{
		RX_isr[THR] = t - _start;
		_RX_good |= _BV(THR);
	}
}
void rud_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_RUD_PIN))
	_start = t;
	else
	{
		RX_isr[RUD] = t - _start;
		_RX_good |= _BV(RUD);
	}
}

void ail_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_AIL_PIN))
	_start = t;
	else
	{
		RX_isr[AIL] = t - _start;
		_RX_good |= _BV(AIL);
	}
}
void ele_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_ELE_PIN))
	_start = t;
	else
	{
		RX_isr[ELE] = t - _start;
		_RX_good |= _BV(ELE);
	}
}

uint16_t RX_ZERO[]={PWM_RC_MID,PWM_RC_MID,PWM_RC_MID,PWM_RC_LOW};
void rxRead()
{
	int b;
	EVERYMS(50)
	{
		RX_good = _RX_good;
		_RX_good = 0;
	}
	for (uint8_t i = 0; i < 4; i++)
	{
		b = RX_isr[i];
		if (b >= 900 && b <= 2100)
		{
			RX_raw[i] = b;
			RX[i] = (int16_t)(RX_raw[i] - RX_ZERO[i]) >> 2;
		}
	}
	for (int i=0; i<3; i++)
	{
		RX[i]=limit(RX[i],-100,100);
		if ( RX[i] >= -NORMAL_CUTOFF && RX[i] <= NORMAL_CUTOFF ) RX[i]=0;
		else if ( RX[i] >  NORMAL_CUTOFF ) RX[i] -=NORMAL_CUTOFF;
		else if ( RX[i] < -NORMAL_CUTOFF ) RX[i] +=NORMAL_CUTOFF;
		
	}

	
	RX[THR]	>>= 1;
	if(RX[THR]<THR_CUTOFF) RX[THR]=0;
	else if (RX[THR]>100) RX[THR]=100;

}


/*

void rxInit()
{
PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
}

ISR(PCINT0_vect){
current_time = micros();
//Channel 1=========================================
if(PINB & B00000001){                                        //Is input 8 high?
if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
last_channel_1 = 1;                                      //Remember current input state
timer_1 = current_time;                                  //Set timer_1 to current_time
}
}
else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
last_channel_1 = 0;                                        //Remember current input state
RX[AIL] = current_time - timer_1;         //Channel 1 is current_time - timer_1
}
//Channel 2=========================================
if(PINB & B00000010 ){                                       //Is input 9 high?
if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
last_channel_2 = 1;                                      //Remember current input state
timer_2 = current_time;                                  //Set timer_2 to current_time
}
}
else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
last_channel_2 = 0;                                        //Remember current input state
RX[ELE] = current_time - timer_2;         //Channel 2 is current_time - timer_2
}
//Channel 3=========================================
if(PINB & B00000100 ){                                       //Is input 10 high?
if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
last_channel_3 = 1;                                      //Remember current input state
timer_3 = current_time;                                  //Set timer_3 to current_time
}
}
else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
last_channel_3 = 0;                                        //Remember current input state
RX[THR] = current_time - timer_3;         //Channel 3 is current_time - timer_3

}
//Channel 4=========================================
if(PINB & B00001000 ){                                       //Is input 11 high?
if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
last_channel_4 = 1;                                      //Remember current input state
timer_4 = current_time;                                  //Set timer_4 to current_time
}
}
else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
last_channel_4 = 0;                                        //Remember current input state
RX[RUD] = current_time - timer_4;         //Channel 4 is current_time - timer_4
}
}
*/