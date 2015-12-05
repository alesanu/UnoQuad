
/*
* IMU.ino
*
* Created: 11/24/2015 1:02:59 PM
*  Author: QuocTuanIT
*/

bool calibrated = false;

double gyroRaw[3];
double gyroRate[3];
double gyroCal[3];

void gyroInit()
{
	Wire.begin();						//start the I2C master.
	digitalWrite(LED_PIN,HIGH);
	delay(1500);
	
	Wire.beginTransmission(0x69);
	Wire.write(0x20);					//register 1 (0x20)
	Wire.write(0x0F);					//turn on the gyro and enable all axis
	Wire.endTransmission();

	Wire.beginTransmission(0x69);
	Wire.write(0x23);					//register 4 (0x23)
	Wire.write(0x90);					//Block Data Update active & 500dps full scale
	Wire.endTransmission();
}

void gyroCalibration()
{
	//calibration
	for (int i = 0; i < 2000 ; i ++) //2000 sample
	{
		if(i % 15 == 0)digitalWrite(LED_PIN, !digitalRead(LED_PIN));
		gyroReadRaw();                                     
		gyroCal[ROL] += gyroRaw[ROL];                              
		gyroCal[PIT] += gyroRaw[PIT];   
		gyroCal[YAW] += gyroRaw[YAW];        
		delay(1);                      
	}
	//Average gyro offset.
	gyroCal[ROL] /= 2000;
	gyroCal[PIT] /= 2000;
	gyroCal[YAW] /= 2000;
	calibrated = true;
}

void gyroReadRaw()
{
	byte highByte, lowByte;
	Wire.beginTransmission(0x69);
	Wire.write(168);
	Wire.endTransmission();
	Wire.requestFrom(0x69, 6);
	while(Wire.available() < 6);
	lowByte = Wire.read();
	highByte = Wire.read();
	gyroRaw[ROL] = ((highByte<<8)|lowByte);
	lowByte = Wire.read();
	highByte = Wire.read();
	gyroRaw[PIT] = ((highByte<<8)|lowByte);
	gyroRaw[PIT] = - gyroRaw[PIT];
	lowByte = Wire.read();
	highByte = Wire.read();
	gyroRaw[YAW] = ((highByte<<8)|lowByte);
	gyroRaw[YAW] = -gyroRaw[YAW];
	if(calibrated)
	{
		gyroRaw[ROL] -= gyroCal[ROL];
		gyroRaw[PIT] -= gyroCal[PIT];
		gyroRaw[YAW] -= gyroCal[YAW];
	}
}
void gyroCaculate()
{
	gyroRate[ROL] = (gyroRate[ROL] * 0.8) + ((gyroRaw[ROL] / 57.14286) * 0.2);  //Gyro pid input is deg/sec.
	gyroRate[PIT] = (gyroRate[PIT] * 0.8) + ((gyroRaw[PIT] / 57.14286) * 0.2); 
	gyroRate[YAW] = (gyroRate[YAW] * 0.8) + ((gyroRaw[YAW] / 57.14286) * 0.2);  
}