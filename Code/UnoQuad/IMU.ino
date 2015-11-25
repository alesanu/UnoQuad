
/*
* IMU.ino
*
* Created: 11/24/2015 1:02:59 PM
*  Author: QuocTuanIT
*/

int cal_int;

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
		if(cal_int % 15 == 0)digitalWrite(LED_PIN, !digitalRead(LED_PIN));   //Change the led status to indicate calibration.
		gyroReadRaw();                                           //Read the gyro output.
		gyroCal[ROL] += gyroRaw[ROL];                                //Ad roll value to gyroCal[ROL].
		gyroCal[PIT] += gyroRaw[PIT];                              //Ad pitch value to gyroCal[PIT].
		gyroCal[YAW] += gyroRaw[YAW];                                  //Ad yaw value to gyroCal[Yaw].
		//We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
		digitalWrite(ESC1_PIN,HIGH); digitalWrite(ESC2_PIN,HIGH); digitalWrite(ESC3_PIN,HIGH); digitalWrite(ESC4_PIN,HIGH);                                        //Set digital poort 4, 5, 6 and 7 high.
		delayMicroseconds(1000);                                   //Wait 1000us.
		digitalWrite(ESC1_PIN,LOW); digitalWrite(ESC2_PIN,LOW); digitalWrite(ESC3_PIN,LOW); digitalWrite(ESC4_PIN,LOW);                                        //Set digital poort 4, 5, 6 and 7 low.
		delay(3);                                                  //Wait 3 milliseconds before the next loop.
	}
	//Average gyro offset.
	gyroCal[ROL] /= 2000;
	gyroCal[PIT] /= 2000;
	gyroCal[YAW] /= 2000;
}

void gyroReadRaw()
{
	byte highByte, lowByte;
	Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
	Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
	Wire.endTransmission();                                      //End the transmission
	Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
	while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
	lowByte = Wire.read();                                       //First received byte is the low part of the angular data
	highByte = Wire.read();                                      //Second received byte is the high part of the angular data
	gyroRaw[ROL] = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
	if(cal_int == 2000)gyroRaw[ROL] -= gyroCal[ROL];               //Only compensate after the calibration
	lowByte = Wire.read();                                       //First received byte is the low part of the angular data
	highByte = Wire.read();                                      //Second received byte is the high part of the angular data
	gyroRaw[PIT] = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
	gyroRaw[PIT] *= -1;                                            //Invert axis
	if(cal_int == 2000)gyroRaw[PIT] -= gyroCal[PIT];             //Only compensate after the calibration
	lowByte = Wire.read();                                       //First received byte is the low part of the angular data
	highByte = Wire.read();                                      //Second received byte is the high part of the angular data
	gyroRaw[YAW] = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
	gyroRaw[YAW] *= -1;                                              //Invert axis
	if(cal_int == 2000)gyroRaw[YAW] -= gyroCal[YAW];                 //Only compensate after the calibration
}
void gyroCaculate()
{
	gyroRate[ROL] = (gyroRate[ROL] * 0.8) + ((gyroRaw[ROL] / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
	gyroRate[PIT] = (gyroRate[PIT] * 0.8) + ((gyroRaw[PIT] / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
	gyroRate[YAW] = (gyroRate[YAW] * 0.8) + ((gyroRaw[YAW] / 57.14286) * 0.2);               //Gyro pid input is deg/sec.
}