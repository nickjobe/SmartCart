#include "servo.h"
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>


void init_shared_variable(SharedVariable* sv) {
	sv->programExit = 0;
	sv->servoPos = 0;
}


void init_actuators(SharedVariable* sv) {
	
	softPwmCreate(SERVO_PIN, 0, 0xff);
}


void servo_motion(SharedVariable* sv) {

	softPwmWrite(SERVO_PIN, sv->servoPos);
	delay(15);
	softPwmWrite(SERVO_PIN, 0);
}

