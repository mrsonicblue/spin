#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <phidget22.h>

int main()
{
	printf("HELLO WORLD!!!\n");

	PhidgetDCMotorHandle dcMotor0;

	PhidgetDCMotor_create(&dcMotor0);

	Phidget_openWaitForAttachment((PhidgetHandle)dcMotor0, 5000);

	PhidgetDCMotor_setTargetVelocity(dcMotor0, 0.25);

	//Wait until Enter has been pressed before exiting
	getchar();

	Phidget_close((PhidgetHandle)dcMotor0);

	PhidgetDCMotor_delete(&dcMotor0);

	return 0;
}
