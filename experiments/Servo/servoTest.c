
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "servo.h"


int main(int argc, char* argv[]) {

	SharedVariable v;

	if (wiringPiSetup() == -1) {
		printf("Failed to setup wiringPi.\n");
		return 1;
	}

	init_shared_variable(&v);
	init_actuators(&v);


	int count = 1000;

	while (v.programExit != 1 && count > 0) {
		
		
		v.servoPos = 255;
		servo_motion(&v);
		delay(10);

		//v.servoPos = 0;
		//servo_motion(&v);
		//delay(10);

		count--;
	}
}
