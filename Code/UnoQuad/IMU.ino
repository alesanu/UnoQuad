
/*
* IMU.ino
*
* Created: 11/24/2015 1:02:59 PM
*  Author: QuocTuanIT
*/

void gyroInit()
{
	Wire.begin();                                                //Start the I2C as master.
	digitalWrite(LED_PIN,HIGH);                                  //Turn on the warning led.
	delay(3000);                                                 //Wait 3 second befor continuing.
	
	Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
	Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
	Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
	Wire.endTransmission();                                      //End the transmission with the gyro

	Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
	Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
	Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
	Wire.endTransmission();                                      //End the transmission with the gyro
}

void gyroCalibration()
{
	//Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	for (cal_int = 0; cal_int < 2000 ; cal_int ++) //Take 2000 readings for calibration.
	{              
		if(cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
		gyroReadRaw();                                           //Read the gyro output.
		gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
		gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
		gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
		//We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
		digitalWrite(ESC1_PIN,HIGH); digitalWrite(ESC2_PIN,HIGH); digitalWrite(ESC3_PIN,HIGH); digitalWrite(ESC4_PIN,HIGH);                                        //Set digital poort 4, 5, 6 and 7 high.
		delayMicroseconds(1000);                                   //Wait 1000us.
		digitalWrite(ESC1_PIN,LOW); digitalWrite(ESC2_PIN,LOW); digitalWrite(ESC3_PIN,LOW); digitalWrite(ESC4_PIN,LOW);                                        //Set digital poort 4, 5, 6 and 7 low.
		delay(3);                                                  //Wait 3 milliseconds before the next loop.
	}
	//Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
	gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
	gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
	gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
}

void gyroReadRaw()
{
	Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
	Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
	Wire.endTransmission();                                      //End the transmission
	Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
	while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
	lowByte = Wire.read();                                       //First received byte is the low part of the angular data
	highByte = Wire.read();                                      //Second received byte is the high part of the angular data
	gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
	if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
	lowByte = Wire.read();                                       //First received byte is the low part of the angular data
	highByte = Wire.read();                                      //Second received byte is the high part of the angular data
	gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
	gyro_pitch *= -1;                                            //Invert axis
	if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
	lowByte = Wire.read();                                       //First received byte is the low part of the angular data
	highByte = Wire.read();                                      //Second received byte is the high part of the angular data
	gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
	gyro_yaw *= -1;                                              //Invert axis
	if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
}
void gyroCaculate()
{
	gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
	gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
	gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.
}