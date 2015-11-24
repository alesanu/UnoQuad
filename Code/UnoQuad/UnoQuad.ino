
#include <Wire.h>
#include "global.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 25;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;



state_t State;

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

void arm(uint8_t value)
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

void armingLoop()
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
	//	 (4)\   /(1)
	//        \ /              x
	//         X		       |
	//        / \         y____|
	//  (3)/   \(2)
	//
	//region of mixer PID Roll-Pitch-Yaw-Throttle
	if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
	escPwm[1] = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
	escPwm[2] = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
	escPwm[3] = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
	escPwm[4] = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
	
	pwmInit();
	pinMode(LED_PIN,OUTPUT);
	delay(3000);                                                 //Wait 2 second befor continuing.
	
	gyroInit();

	delay(250);                                                  //Give the gyro time to start.

	gyroCalibration();
	
	rxInit();

	
	//Load the battery voltage to the battery_voltage variable.
	//65 is the voltage compensation for the diode.
	//12.6V equals ~5V @ Analog 0.
	//12.6V equals 1023 analogRead(0).
	//1260 / 1023 = 1.2317.
	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	battery_voltage = (analogRead(0) + 65) * 1.2317;
	
	//When everything is done, turn off the led.
	digitalWrite(LED_PIN,HIGH);                                        //Turn off the warning led.
	Serial.begin(115200);
	Serial.println("Init Success");
}



void loop()
{
	gyroReadRaw();
	gyroCaculate();
	rxRead();
	checkState();
	armingLoop();
	

	calculate_pid();
	
	//The battery voltage is needed for compensation.
	//A complementary filter is used to reduce noise.
	//0.09853 = 0.08 * 1.2317.
	battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
	
	//Turn on the led if battery voltage is to low.
	if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(12, HIGH);

	
	if (State.Armed && !State.ThrottleOff)
	{                                                          //The motors are started.
		mixers(RX_raw[THR]);
		pid_roll_setpoint = RX[AIL]/3.5;
		pid_pitch_setpoint = -RX[ELE]/3.5;
		pid_yaw_setpoint = RX[RUD]/3.5;
	}
	else
	{
		pid_roll_setpoint = 0;
		pid_pitch_setpoint = 0;
		pid_yaw_setpoint = 0;
		escPwm[1] = escPwm[2] = escPwm[3] = escPwm[4] = 1000;
	}

	pwmOutput();
	
	
	#ifdef OUT_RX
	Serial.print("AIL: "); Serial.print(RX[AIL]); Serial.print("\t");
	Serial.print("ELE: "); Serial.print(RX[ELE]); Serial.print("\t");
	Serial.print("RUD: "); Serial.print(RX[RUD]); Serial.print("\t");
	Serial.print("THR: "); Serial.print(RX_raw[THR]); Serial.print("\n");
	
	#endif
	
}

void calculate_pid(){
	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
	
	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
	
	pid_last_roll_d_error = pid_error_temp;
	
	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
	
	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
	
	pid_last_pitch_d_error = pid_error_temp;
	
	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
	
	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
	
	pid_last_yaw_d_error = pid_error_temp;
}
void pidReset()
{
	//Reset the pid controllers for a bumpless start.
	pid_i_mem_roll = 0;
	pid_last_roll_d_error = 0;
	pid_i_mem_pitch = 0;
	pid_last_pitch_d_error = 0;
	pid_i_mem_yaw = 0;
	pid_last_yaw_d_error = 0;
	pid_roll_setpoint = 0;
	pid_pitch_setpoint = 0;
	pid_yaw_setpoint = 0;
}