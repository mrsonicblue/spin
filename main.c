#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <sys/time.h>
#include <math.h>
#include <phidget22.h>

#define ACCELERATION 1.0
#define SENSITIVITY_FULL 0.0
#define SENSITIVITY_NONE 1.0
#define VELOCITY1 0.4
#define VELOCITY2 0.1
#define FILTER_DEPTH 5
#define ERROR 0.0

pthread_mutex_t _lock;
PhidgetDCMotorHandle _motor;
PhidgetAccelerometerHandle _accelerometer;

static volatile bool _terminated;
bool _primed;
bool _working;
int _prime;
double _xFilter[FILTER_DEPTH], _yFilter[FILTER_DEPTH], _zFilter[FILTER_DEPTH];
double _xCoord, _yCoord, _zCoord;
double _absoluteAngle;
double _relativeAngle;
int _rotations;
double _targetRequest;
double _target;
struct timeval _timeout;
struct timeval _halfSecond;

void setAcceleratorSensitivity(double sensitivity)
{
	PhidgetAccelerometer_setAccelerationChangeTrigger(_accelerometer, sensitivity);
}

double fmin(double a, double b)
{
	return (a < b) ? a : b;
}

void shutDown()
{
	if (_terminated)
		return;

	_working = false;
	_terminated = true;
}

void turnTo(double angle)
{
	if (_terminated)
	 	return;

	pthread_mutex_lock(&_lock);

	setAcceleratorSensitivity(SENSITIVITY_FULL);
	_targetRequest = fmod(angle, 360.0);

	pthread_mutex_unlock(&_lock);
}

double processFilter(double * values, double value)
{
	double result = value;
	for (int i = 0; i < FILTER_DEPTH - 1; i++)
	{
		values[i] = values[i + 1];
		result += values[i];
	}

	values[FILTER_DEPTH - 1] = value;

	return result / (double)FILTER_DEPTH;
}

double calculateAngle()
{
	return atan2(_xCoord, _yCoord) * 180.0 / M_PI + 180.0 + ERROR;
}

static void CCONV onMotor_Attach(PhidgetHandle ch, void * ctx)
{
}

static void CCONV onMotor_Detach(PhidgetHandle ch, void * ctx) 
{
	shutDown();
}

static void CCONV onMotor_Error(PhidgetHandle ch, void * ctx, Phidget_ErrorEventCode code, const char * description)
{
	shutDown();
}

static void CCONV onAccelerometer_Attach(PhidgetHandle ch, void * ctx) 
{
}

static void CCONV onAccelerometer_Detach(PhidgetHandle ch, void * ctx)
{
	shutDown();
}

static void CCONV onAccelerometer_Error(PhidgetHandle ch, void * ctx, Phidget_ErrorEventCode code, const char * description)
{
	shutDown();
}

static void CCONV onAccelerometer_AccelerationChange(PhidgetAccelerometerHandle ch, void * ctx, const double acceleration[3], double timestamp)
{
	if (_terminated)
		return;

	pthread_mutex_lock(&_lock);

	printf("Acceleration: \t%lf  |  %lf  |  %lf\n", acceleration[0], acceleration[1], acceleration[2]);
	printf("Timestamp: %lf\n", timestamp);
	printf("----------\n");

	_xCoord = processFilter(_xFilter, acceleration[0]);
	_yCoord = processFilter(_yFilter, acceleration[1]);
	_zCoord = processFilter(_zFilter, acceleration[2]);

	if (_prime > 0)
		_prime--;

	// Handle priming
	if (!_primed)
	{
		if (_prime > 0)
		{
			pthread_mutex_unlock(&_lock);
			return;
		}

		// Initialize rotation count
		_relativeAngle = calculateAngle();

		if (_relativeAngle > 270)
			_rotations = -1;

		setAcceleratorSensitivity(SENSITIVITY_NONE);

		_primed = true;
		printf("primed!\n");
	}

	// Handle target request
	if (_targetRequest != -999.0)
	{
		if (_target != _targetRequest)
		{
			_target = _targetRequest;
			timerclear(&_timeout);

			setAcceleratorSensitivity(SENSITIVITY_FULL);

			_working = true;
		}

		_targetRequest = -999.0;
	}

	double relativeAngleWas = _relativeAngle;
	_relativeAngle = calculateAngle();

	// Update rotation count
	double rawAngleDelta = relativeAngleWas - _relativeAngle;
	if (fabs(rawAngleDelta) > 300.0)
	{
		_rotations += (rawAngleDelta >= 0.0) ? 1 : -1;
	}

	_absoluteAngle = _relativeAngle + (double)(_rotations * 360);

	if (!_working) {
		pthread_mutex_unlock(&_lock);
		return;
	}

	struct timeval now;
	gettimeofday(&now, NULL);

	// Check rotation progress
	int _direction = (_target < _absoluteAngle) ? 1 : -1;

	double diff = fmin(fabs(_target - _absoluteAngle), 10.0);

	printf("A: %lf  D: %lf\n", _absoluteAngle, diff);

	// We're within a degree
	if (diff < 1.0)
	{
		if (!timerisset(&_timeout))
		{
			// Stop and wait for 0.5 seconds before declaring success
			PhidgetDCMotor_setTargetVelocity(_motor, 0.0);
			timeradd(&now, &_halfSecond, &_timeout);
			printf("waiting a half sec\n");
		}
		else
		{
			if (timercmp(&_timeout, &now, <))
			{
				// Success
				setAcceleratorSensitivity(SENSITIVITY_NONE);
				_working = false;
				printf("WE THERE\n");
			}
		}
	}
	else
	{
		timerclear(&_timeout);

		double velocity = diff * (VELOCITY1 - VELOCITY2) / 10.0 + VELOCITY2;
		velocity *= _direction;
		printf("V: %lf\n", velocity);

		double currentVelocity;
		PhidgetDCMotor_getTargetVelocity(_motor, &currentVelocity);
		if (currentVelocity != velocity)
		{
			PhidgetDCMotor_setTargetVelocity(_motor, velocity);
		}
	}

	pthread_mutex_unlock(&_lock);
}

void intHandler(int dummy)
{
    shutDown();
}

int main(int argc, char *argv[])
{
	double initialAngle = 0.0;
	if (argc > 1)
		initialAngle = atof(argv[1]);

	printf("Initial angle is %lf\n", initialAngle);

	if (pthread_mutex_init(&_lock, NULL) != 0)
	{ 
        printf("mutex init has failed\n"); 
        return 1;
    } 

	_terminated = false;
	_primed = false;

	_working = false;
	_prime = FILTER_DEPTH;
	_xCoord = 0.0;
	_yCoord = 0.0;
	_zCoord = 0.0;
	_absoluteAngle = 0.0;
	_relativeAngle = 0.0;
	_targetRequest = -999.0;
	_target = -1.0;
	_rotations = 0;
	timerclear(&_timeout);
	_halfSecond.tv_sec = 0;
	_halfSecond.tv_usec = 500000;

	signal(SIGINT, intHandler);

	PhidgetDCMotor_create(&_motor);
	Phidget_setOnAttachHandler((PhidgetHandle)_motor, onMotor_Attach, NULL);
	Phidget_setOnDetachHandler((PhidgetHandle)_motor, onMotor_Detach, NULL);
	Phidget_setOnErrorHandler((PhidgetHandle)_motor, onMotor_Error, NULL);

	Phidget_openWaitForAttachment((PhidgetHandle)_motor, 500);
	PhidgetDCMotor_setAcceleration(_motor, ACCELERATION);

	PhidgetAccelerometer_create(&_accelerometer);
	Phidget_setOnAttachHandler((PhidgetHandle)_accelerometer, onAccelerometer_Attach, NULL);
	Phidget_setOnDetachHandler((PhidgetHandle)_accelerometer, onAccelerometer_Detach, NULL);
	Phidget_setOnErrorHandler((PhidgetHandle)_accelerometer, onAccelerometer_Error, NULL);
	PhidgetAccelerometer_setOnAccelerationChangeHandler(_accelerometer, onAccelerometer_AccelerationChange, NULL);

	Phidget_openWaitForAttachment((PhidgetHandle)_accelerometer, 500);
	PhidgetAccelerometer_setDataInterval(_accelerometer, 16);

	turnTo(initialAngle);

	while (!_terminated) 
	{
		printf("PING\n");

		sleep(1);
	}

	printf("\n");
	printf("Cleaning up!\n");

	pthread_mutex_lock(&_lock);

	PhidgetDCMotor_setTargetVelocity(_motor, 0.0);

	Phidget_close((PhidgetHandle)_motor);
	PhidgetDCMotor_delete(&_motor);

	Phidget_close((PhidgetHandle)_accelerometer);
	PhidgetAccelerometer_delete(&_accelerometer);

	pthread_mutex_unlock(&_lock);
	pthread_mutex_destroy(&_lock);

	printf("\n");
	printf("All done!\n");

	return 0;
}