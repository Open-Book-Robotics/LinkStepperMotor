#include "LinkStepperMotor.hpp"
#include <Arduino.h>

void LinkStepperMotor::initialize() {
	// Setup pins
	pinMode(this->stepPin, OUTPUT);
	pinMode(this->dirPin, OUTPUT);
	pinMode(this->enablePin, OUTPUT);

	// Set initial pin states
	digitalWrite(this->stepPin, LOW);
	digitalWrite(this->dirPin, LOW);

	this->setSpeedRPM(30);

	this->enable();
}

void LinkStepperMotor::enable() { digitalWrite(this->enablePin, LOW); }
void LinkStepperMotor::disable() { digitalWrite(this->enablePin, HIGH); }
bool LinkStepperMotor::isEnabled() { return !digitalRead(this->enablePin); }

int16_t LinkStepperMotor::getCurrentPosition() { return this->currentPosition; }
int16_t LinkStepperMotor::getTargetPosition() { return this->targetPosition; }
float LinkStepperMotor::getCurrentAngle() { return this->currentAngle; }
uint16_t LinkStepperMotor::getSpeedSPS() { return this->currentSpeedSPS; }
unsigned long LinkStepperMotor::getPulseInterval() { return this->currentPulseInterval; }

void LinkStepperMotor::calibrate(bool calibrationDirection, bool calibrationNO) {
	if (this->calibrationPin == -1) {
		// Logger::debug("Calibration Pin not set, skipping calibration");
		return;
	}
	// Set the calibration pin to the correct mode
	if (calibrationNO) {
		pinMode(this->calibrationPin, INPUT_PULLUP);
	} else {
		pinMode(this->calibrationPin, INPUT);
	}
	// Set the calibration direction and speed
	calibrationDirection ? digitalWrite(this->dirPin, HIGH) : digitalWrite(this->dirPin, LOW);
	float originalSpeedRPM = this->currentSpeedRPM;
	this->setSpeedRPM(this->currentSpeedRPM * 0.5f);

	// Wait for the calibration pin to be pressed
	int initialCalibrationPinState = digitalRead(calibrationPin);
	while (digitalRead(calibrationPin) != initialCalibrationPinState) {
		this->stepMotor();
	}

	// Reset the link speed
	this->setSpeedRPM(originalSpeedRPM);

	// Set the current position to 0
	this->currentPosition = 0;
	this->currentAngle = 0;
}

void LinkStepperMotor::update() {
	// If the motor is not moving, we have reached the target
	if (!this->isMoving()) { return; }

	// Determine the time since the last step
	unsigned long currentTime = micros();
	unsigned long timeSinceLastStep = currentTime - this->previousModulationTime;
	if (timeSinceLastStep > this->currentPulseInterval) {
		// Swap HIGH/LOW pulse on every occurance of the delay
		this->isRunning = !this->isRunning;
		this->isRunning ? digitalWrite(this->stepPin, HIGH) : digitalWrite(this->stepPin, LOW);
		// Increment/Decrement steps during LOW pulse in order to avoid missing steps
		if (!this->isRunning && this->currentPosition != this->targetPosition) {
			this->currentPosition < this->targetPosition ? this->currentPosition++ : this->currentPosition--;
		}
		this->previousModulationTime = currentTime;
	}
}

void LinkStepperMotor::updateTrapezoidal() {
	// If the motor is not moving, we have reached the target
	if (!this->isMoving()) { return; }

	// Determine the time since the last step
	unsigned long currentTime = micros();
	unsigned long timeSinceLastStep = currentTime - this->previousModulationTime;
	if (timeSinceLastStep > this->currentPulseInterval) {
		// Swap HIGH/LOW pulse on every occurance of the delay
		this->isRunning = !this->isRunning;
		this->isRunning ? digitalWrite(this->stepPin, HIGH) : digitalWrite(this->stepPin, LOW);
		// Increment/Decrement steps during LOW pulse in order to avoid missing steps
		if (!this->isRunning && this->currentPosition != this->targetPosition) {
			this->currentPosition < this->targetPosition ? this->currentPosition++ : this->currentPosition--;
		}
		this->previousModulationTime = currentTime;
		// Speed can be reassigned here to compute a new currentPulseInterval depending on the progress of the movement
		this->computeNewPulseIntervalTrapezoidal();
	}
}

void LinkStepperMotor::updateAccel() {
	// If the motor is not moving, we have reached the target
	if (!this->isMoving()) { return; }

	// Determine the time since the last step
	unsigned long currentTime = micros();
	unsigned long timeSinceLastStep = currentTime - this->previousModulationTime;
	if (timeSinceLastStep > this->currentPulseInterval) {
		// Swap HIGH/LOW pulse on every occurance of the delay
		this->isRunning = !this->isRunning;
		this->isRunning ? digitalWrite(this->stepPin, HIGH) : digitalWrite(this->stepPin, LOW);
		// Increment/Decrement steps during LOW pulse in order to avoid missing steps
		if (!this->isRunning && this->currentPosition != this->targetPosition) {
			this->currentPosition < this->targetPosition ? this->currentPosition++ : this->currentPosition--;
		}
		this->previousModulationTime = currentTime;
		// Speed can be reassigned here to compute a new currentPulseInterval depending on the progress of the movement
		this->computeNewPulseInterval();
	}
}

void LinkStepperMotor::computeNewPulseIntervalTrapezoidal() {
	// Acceleration curve is split into 3 parts: acceleration, steady-state, deceleration
	// TODO: inline functions for efficiency
	int stepsRemaining = this->getStepsRemaining();
	int stepsCompleted = this->getStepsCompleted();
	int stepsAccel = stepsRemaining * 0.4f; // same as stepsDecel
	int stepsMax = stepsRemaining * 0.2f;
	float accelConstant = (this->maxSpeedSPS - this->minSpeedSPS) / (stepsAccel);

	// Determine which range we are in to apply the correct part of the acceleration curve
	if (stepsCompleted < stepsAccel) {
		int16_t speedSPS = (accelConstant * stepsCompleted) + minSpeedSPS;
		this->setSpeedSPS(speedSPS);
	} else if (stepsCompleted >= stepsAccel && stepsCompleted <= (stepsAccel + stepsMax)) {
		this->setSpeedSPS(this->maxSpeedSPS);
	} else if (stepsCompleted > (stepsAccel + stepsMax) && stepsCompleted <= stepsRemaining) {
		int16_t speedSPS = (accelConstant * (stepsRemaining - stepsCompleted)) + minSpeedSPS;
		this->setSpeedSPS(speedSPS);
	} else { // Shouldn't happen
		this->setSpeedSPS(this->minSpeedSPS);
	}
}

void LinkStepperMotor::computeNewPulseInterval() {
	// Acceleration curve is split into 3 parts: acceleration, steady-state, deceleration
	int totalSteps = abs(this->targetPosition - this->previousTargetPosition);
	int stepsRemaining = this->getStepsRemaining();
	int stepsCompleted = totalSteps - stepsRemaining;
	int n1 = totalSteps / 3;
	int n2 = 2 * n1;
	uint16_t speedSPS = this->currentSpeedSPS;
	// Determine which range we are in to apply the correct part of the acceleration curve
	if (stepsCompleted <= n1) {
		// Acceleration
		// a(n) = k * n1 + a0
		// v(n) = 0.5 * k * n^2 + a0 * n + v0
		speedSPS = (0.5f * this->accelerationRate * pow(stepsCompleted, 2)) + (this->initialAcceleration * stepsCompleted) + this->initialSpeedSPS;
	} else if (stepsCompleted >= n1 && stepsCompleted <= n2) {
		// Steady-state
		// a(n) = a_max = k * n1 + a0
		// v(n) = (k * n1 + a0) * n - 0.5 * k * n1^2 + v0
		speedSPS = ((this->accelerationRate * n1 + this->initialAcceleration) * stepsCompleted) - (0.5f * this->accelerationRate * pow(n1, 2)) + this->initialSpeedSPS;
	} else { // (stepsCompleted >= n2)
		// Deceleration
		// a(n) = -k * n + k * n2 + a_max
		// v(n) = -0.5 * k * n^2 + (k * n1 + k * n2 + a0) * n - 0.5 * k * (n1^2 + n2^2) + v0
		speedSPS = (-0.5f * this->accelerationRate * pow(stepsCompleted, 2))
			+ (this->accelerationRate * n1 + this->accelerationRate * n2 + this->initialAcceleration) * stepsCompleted
			- (0.5f * this->accelerationRate * (pow(n1, 2) + pow(n2, 2)))
			+ this->initialSpeedSPS;
	}
	this->setSpeedSPS(speedSPS);
}

void LinkStepperMotor::stepMotor() {
	digitalWrite(this->stepPin, HIGH);
	delayMicroseconds(this->currentPulseInterval);
	digitalWrite(this->stepPin, LOW);
	delayMicroseconds(this->currentPulseInterval);
}

bool LinkStepperMotor::isMoving() { return this->currentPosition != this->targetPosition; }

void LinkStepperMotor::updateCurrentAngle() {
	this->currentAngle = (this->currentPosition / ((float)this->stepsPerRevolution * this->gearRatio)) * 360.0f;
}

void LinkStepperMotor::setTargetPosition(int16_t targetPosition) {
	if (isMoving()) { return; } // Can't set new target position if the motor is already moving
	this->previousTargetPosition = this->currentPosition;
	this->targetPosition = targetPosition;
	this->currentDirection = this->targetPosition < this->currentPosition; // true if CW, false if CCW
	this->setDirection(this->currentDirection);
}

bool LinkStepperMotor::isInRangeOfMotion(float angleDegrees) {
	return angleDegrees >= this->minAngle && angleDegrees <= this->maxAngle;
}

void LinkStepperMotor::setTargetPositionDegrees(float targetAngleDegrees) {
	if (!isInRangeOfMotion(targetAngleDegrees)) {
		// Flip the angle if it is out of range as well as the direction
		targetAngleDegrees = targetAngleDegrees - 360.0f;
		// If the angle is still out of range, return
		if (!isInRangeOfMotion(targetAngleDegrees)) { return; }
	}
	int16_t targetPosition = this->convertDegreesToSteps(targetAngleDegrees);
	this->setTargetPosition(targetPosition);
}

void LinkStepperMotor::setSpeedRPM(float speedRPM) {
	uint16_t linkSPS = (speedRPM / 60.0f) * this->stepsPerRevolution * this->gearRatio;
	this->setSpeedSPS(linkSPS);
}

void LinkStepperMotor::setSpeedSPS(uint16_t speedSPS) {
	this->currentSpeedSPS = speedSPS;
	this->currentPulseInterval = this->getPulseIntervalFromSpeed(speedSPS);
}

void LinkStepperMotor::setDirection(bool CW) {
	CW ? digitalWrite(this->dirPin, HIGH) : digitalWrite(this->dirPin, LOW);
	this->currentDirection = CW;
}

inline int16_t LinkStepperMotor::convertDegreesToSteps(float degrees) { return round((degrees / 360.0f) * this->stepsPerRevolution * this->gearRatio); }
inline uint16_t LinkStepperMotor::getStepsRemaining() { return abs(this->targetPosition - this->currentPosition); }
inline uint16_t LinkStepperMotor::getStepsCompleted() { return abs(this->currentPosition - this->previousTargetPosition); }
inline uint16_t LinkStepperMotor::getTotalStepsToTarget() { return abs(this->targetPosition - this->previousTargetPosition); }