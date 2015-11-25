/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino/Genuino Uno, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 165
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 165
#define ARDUINO_AVR_UNO
#define ARDUINO_ARCH_AVR
extern "C" void __cxa_pure_virtual() {;}

static void checkState();
static void arm(uint8_t value);
static void armingLoop();
int batVoltage();
void mixers(int throttle);
//
//
void commandInit();
void unrecognized();
void LED_on();
void LED_off();
void soft_reset();
void arm_control();
void thr_control();
void imu_angle();
void cal_angle();
void setMax();
void setP();
void setI();
void setD();
void gyroInit();
void gyroCalibration();
void gyroReadRaw();
void gyroCaculate();
void pidInit();
void calculate_pid();
void pidReset();
void pwmInit();
void pwmOutput();
void rxInit();
void thr_callback();
void rud_callback();
void ail_callback();
void ele_callback();
void rxRead();

#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include <UnoQuad.ino>
#include <CMD.ino>
#include <IMU.ino>
#include <PID.ino>
#include <PWM.ino>
#include <PinChangeInt.h>
#include <RX.ino>
#include <SerialCommand.cpp>
#include <SerialCommand.h>
#include <global.h>
