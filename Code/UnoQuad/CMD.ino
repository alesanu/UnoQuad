/*
* CMD.ino
*
* Created: 11/15/2015 9:50:07 AM
*  Author: QuocTuanIT
*/


void commandInit()
{
	command.addCommand("on",LED_on);
	command.addCommand("off",LED_off);
	command.addCommand("rst",soft_reset);
	command.addCommand("arm",arm_control);
	command.addCommand("thr",thr_control);
	command.addCommand("imu",imu_angle);
	command.addCommand("cal",cal_angle);
	command.addCommand("max",setMax);
	command.addCommand("p",setP);
	command.addCommand("i",setI);
	command.addCommand("d",setD);
	command.addDefaultHandler(unrecognized);
}
void unrecognized()
{
	Serial.println("WRONG CMD!!");
}
void LED_on()
{
	Serial.println("LED on");
	digitalWrite(LED_PIN,LOW);
}
void LED_off()
{
	Serial.println("LED off");
	digitalWrite(LED_PIN,HIGH);
}
void soft_reset()
{
	do
	{
		wdt_enable(WDTO_15MS);
		for(;;)
		{
		}
	} while(0);
}
void arm_control()
{
	int param;
	Serial.print("ARM: ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atoi(arg);
		if (param) { arm(ON); Serial.println("ON");}
		else { arm(OFF); Serial.println("OFF");}
	}
	else Serial.println("??? param");
}
void thr_control()
{
	int param;
	Serial.print("THR: ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atoi(arg);
		RX[THR] = param;
		Serial.println(param);
	}
	else
	{
		Serial.println(RX[THR]);
	}
}
void imu_angle()
{
	Serial.print("rollRate: ");  Serial.print(gyroRate[ROL]); Serial.print("\t");
	Serial.print("pitchRate: "); Serial.print(gyroRate[PIT]); Serial.print("\t");
	Serial.print("yawRate: "); Serial.println(gyroRate[YAW]);
}
void cal_angle()
{
	Serial.print("rollCal: "); Serial.print(gyroCal[ROL]); Serial.print("\t");
	Serial.print("pitchCal: "); Serial.print(gyroCal[PIT]); Serial.print("\t");
	Serial.print("yawCal: "); Serial.println(gyroCal[YAW]);
}

void setMax()
{
	float param;
	Serial.print("Set Out Max ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial.println(param);
		pid.roll.max = param;
	}
	else Serial.println("??? param");
}
void setP()
{
	float param;
	Serial.print("set p ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial.println(param,4);
		pid.roll.Kp = param;		
	}
	else Serial.println("??? param");
}
void setI()
{
	float param;
	Serial.print("Set I ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial.println(param,4);
		pid.roll.Ki = param;
	}
	else Serial.println("??? param");
}
void setD()
{
	float param;
	Serial.print("Set D ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial.println(param,4);
		pid.roll.Kd = param;
	}
	else Serial.println("??? param");
}