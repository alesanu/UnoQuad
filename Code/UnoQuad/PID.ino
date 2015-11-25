/*
 * PID.ino
 *
 * Created: 11/25/2015 10:37:40 AM
 *  Author: QuocTuanIT
 */ 

float pidError;
float lastDErrRoll, lastDErrPitch, lastDErrYaw;

void pidInit()
{
	pid.roll.Kp = 1.3;	//stable: 1.3
	pid.roll.Ki = 0.05;	//stable: 0.05
	pid.roll.Kd = 25;	//stable: 25
	pid.roll.max = 400;	//stable: 400
	
	pid.pitch.Kp = pid.roll.Kp;
	pid.pitch.Ki = pid.roll.Ki;
	pid.pitch.Kd = pid.roll.Kd;
	pid.pitch.max= pid.roll.max;
	
	pid.yaw.Kp = 4.0;  //stable: 4.0
	pid.yaw.Ki = 0.02; //stable: 0.02
	pid.yaw.Kd = 0.0;  //stable: 0
	pid.yaw.max= 400;  //stable: 400
	pidReset();
}

void calculate_pid(){
	//Roll
	pidError = gyroRate[ROL] - setPoint[ROL];
	pidState.roll.iTerm += pid.roll.Ki * pidError;
	if(pidState.roll.iTerm > pid.roll.max)pidState.roll.iTerm = pid.roll.max;
	else if(pidState.roll.iTerm < pid.roll.max * -1)pidState.roll.iTerm = pid.roll.max * -1;
	
	pidOut[ROL] = pid.roll.Kp * pidError + pidState.roll.iTerm + pid.roll.Kd * (pidError - pidState.roll.lastDErr);
	if(pidOut[ROL] > pid.roll.max)pidOut[ROL] = pid.roll.max;
	else if(pidOut[ROL] < pid.roll.max * -1)pidOut[ROL] = pid.roll.max * -1;
	
	pidState.roll.lastDErr = pidError;
	
	//Pitch
	pidError = gyroRate[PIT] - setPoint[PIT];
	pidState.pitch.iTerm += pid.pitch.Ki * pidError;
	if(pidState.pitch.iTerm > pid.pitch.max)pidState.pitch.iTerm = pid.pitch.max;
	else if(pidState.pitch.iTerm < pid.pitch.max * -1)pidState.pitch.iTerm = pid.pitch.max * -1;
	
	pidOut[PIT] = pid.pitch.Kp * pidError + pidState.pitch.iTerm + pid.pitch.Kd * (pidError - pidState.pitch.lastDErr);
	if(pidOut[PIT] > pid.pitch.max)pidOut[PIT] = pid.pitch.max;
	else if(pidOut[PIT] < pid.pitch.max * -1)pidOut[PIT] = pid.pitch.max * -1;
	
	pidState.pitch.lastDErr = pidError;
	
	//Yaw
	pidError = gyroRate[YAW] - setPoint[YAW];
	pidState.yaw.iTerm += pid.yaw.Ki * pidError;
	if(pidState.yaw.iTerm > pid.yaw.max)pidState.yaw.iTerm = pid.yaw.max;
	else if(pidState.yaw.iTerm < pid.yaw.max * -1)pidState.yaw.iTerm = pid.yaw.max * -1;
	
	pidOut[YAW] = pid.yaw.Kp * pidError + pidState.yaw.iTerm + pid.yaw.Kd * (pidError - pidState.yaw.lastDErr);
	if(pidOut[YAW] > pid.yaw.max)pidOut[YAW] = pid.yaw.max;
	else if(pidOut[YAW] < pid.yaw.max * -1)pidOut[YAW] = pid.yaw.max * -1;
	
	pidState.yaw.lastDErr = pidError;
}
void pidReset()
{
	//Reset the pid controllers element
	pidState.roll.iTerm = 0;
	pidState.roll.lastDErr = 0;
	pidState.pitch.iTerm = 0;
	pidState.pitch.lastDErr = 0;
	pidState.yaw.iTerm = 0;
	pidState.yaw.lastDErr = 0;
	setPoint[ROL] = 0;
	setPoint[PIT] = 0;
	setPoint[YAW] = 0;
}