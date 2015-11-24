/*
* PWM.ino
*
* Created: 11/23/2015 3:56:21 PM
*  Author: QuocTuanIT
*/
//
int escPwm[5];
//
//unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
//unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
//unsigned long loop_timer;
//
void pwmInit()
{
	pinMode(ESC1_PIN,OUTPUT);
	pinMode(ESC2_PIN,OUTPUT);
	pinMode(ESC3_PIN,OUTPUT);
	pinMode(ESC4_PIN,OUTPUT);
}
void pwmOutput()
{
	//All the information for controlling the motor's is available.
	//The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
	while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
	loop_timer = micros();                                                    //Set the timer for the next loop.
	digitalWrite(ESC1_PIN,HIGH);
	digitalWrite(ESC2_PIN,HIGH);
	digitalWrite(ESC3_PIN,HIGH);
	digitalWrite(ESC4_PIN,HIGH);                                                       //Set digital outputs 4,5,6 and 7 high.
	timer_channel_1 = escPwm[1] + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
	timer_channel_2 = escPwm[2] + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
	timer_channel_3 = escPwm[3] + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
	timer_channel_4 = escPwm[4] + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
	
	while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
		esc_loop_timer = micros();                                              //Read the current time.
		if(timer_channel_1 <= esc_loop_timer)digitalWrite(ESC1_PIN,LOW);                //Set digital output 4 to low if the time is expired.
		if(timer_channel_2 <= esc_loop_timer)digitalWrite(ESC2_PIN,LOW);                //Set digital output 5 to low if the time is expired.
		if(timer_channel_3 <= esc_loop_timer)digitalWrite(ESC3_PIN,LOW);                //Set digital output 6 to low if the time is expired.
		if(timer_channel_4 <= esc_loop_timer)digitalWrite(ESC4_PIN,LOW);                //Set digital output 7 to low if the time is expired.
	}
}