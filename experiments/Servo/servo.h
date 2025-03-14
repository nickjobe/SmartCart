#ifndef _ASSIGNMENT_BODY_
#define _ASSIGNMENT_BODY_

#include <stdint.h>

// Servo WiringPi Pins
#define SERVO_PIN 26

// Servo Min and Max Outputs
#define SERVO_MIN 0
#define SERVO_MAX 255

// Shared Structure
typedef struct shared_variable {
	int programExit;
	int servoPos;
} SharedVariable;

// Functions

void init_shared_variable(SharedVariable* sv);
void init_actuators(SharedVariable* sv);
void servo_motion(SharedVariable* sv);

#endif
