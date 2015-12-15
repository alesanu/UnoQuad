/*
 * PID.ino
 *
 * Created: 11/25/2015 10:37:40 AM
 *  Author: QuocTuanIT
 */ 



void pidInit()
{
	pid.roll.Kp = 1.3;	//stable: 1.3
	pid.roll.Ki = 0.05;	//stable: 0.05
	pid.roll.Kd = 20;	//stable: 25
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

void calculate_pid()
{
	float pidError;
	//Roll
	pidError = gyroRate[ROL] - setPoint[ROL];
	pidState.roll.iTerm += pid.roll.Ki * pidError;
	pidState.roll.iTerm = limit(pidState.roll.iTerm,-pid.roll.max,pid.roll.max);
	pidOut[ROL] = pid.roll.Kp * pidError + pidState.roll.iTerm + pid.roll.Kd * (pidError - pidState.roll.lastDErr);
	pidOut[ROL] = limit(pidOut[ROL],-pid.roll.max,pid.roll.max);
	pidState.roll.lastDErr = pidError;
	
	//pitch
	pidError = gyroRate[PIT] - setPoint[PIT];
	pidState.pitch.iTerm += pid.pitch.Ki * pidError;
	pidState.pitch.iTerm = limit(pidState.pitch.iTerm, -pid.pitch.max, pid.pitch.max);
	pidOut[PIT] = pid.pitch.Kp * pidError + pidState.pitch.iTerm + pid.pitch.Kd * (pidError - pidState.pitch.lastDErr);
	pidOut[PIT] = limit(pidOut[PIT], -pid.pitch.max, pid.pitch.max);
	pidState.pitch.lastDErr = pidError;
	
	//Yaw
	pidError = gyroRate[YAW] - setPoint[YAW];
	pidState.yaw.iTerm += pid.yaw.Ki * pidError;
	pidState.yaw.iTerm = limit(pidState.yaw.iTerm, -pid.yaw.max, pid.yaw.max);
	pidOut[YAW] = pid.yaw.Kp * pidError + pidState.yaw.iTerm + pid.yaw.Kd * (pidError - pidState.yaw.lastDErr);
	pidOut[YAW] = limit(pidOut[YAW], -pid.yaw.max, pid.yaw.max);
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