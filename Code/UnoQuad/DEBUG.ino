
/*
* DEBUG.ino
*
* Created: 12/4/2015 4:44:34 PM
*  Author: QuocTuanIT
*/
void debugProcess()
{

	#ifdef OUT_RX
	Serial.print("AIL: "); Serial.print(RX[AIL]); Serial.print("\t");
	Serial.print("ELE: "); Serial.print(RX[ELE]); Serial.print("\t");
	Serial.print("RUD: "); Serial.print(RX[RUD]); Serial.print("\t");
	Serial.print("THR: "); Serial.print(RX_raw[THR]); Serial.print("\t");
	Serial.print("AUX: "); Serial.print(RX[AUX]); Serial.print("\n");
	delay(100);
	#endif

	#ifdef OUT_GYRO
	Serial.print("R: "); Serial.print(gyroRate[ROL]); Serial.print("   ");
	Serial.print("P: "); Serial.print(gyroRate[PIT]); Serial.print("   ");
	Serial.print("Y: "); Serial.print(gyroRate[YAW]); Serial.print("\n");
	delay(100);
	#endif
	
	#ifdef OUT_MOTOR
	Serial.print("M1: "); Serial.print(escPwm[1]); Serial.print("\t");
	Serial.print("M2: "); Serial.print(escPwm[2]); Serial.print("\t");
	Serial.print("M3: "); Serial.print(escPwm[3]); Serial.print("\t");
	Serial.print("M4: "); Serial.print(escPwm[4]); Serial.print("\n");
	#endif
}