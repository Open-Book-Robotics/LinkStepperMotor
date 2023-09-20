#include "LinkStepperMotor.hpp"
#include <Arduino.h>

// TODO: Write documentation for all functions
// TODO: Add support for acceleration and deceleration curves
// TODO: Move all implementation to .cpp file and offer .hpp as the only visible file for library
// TODO: Test library with physical hardware

void LinkStepperMotor::initialize() {
	// Setup pins
	pinMode(this->stepPin, OUTPUT);
	pinMode(this->dirPin, OUTPUT);
	pinMode(this->enablePin, OUTPUT);

	this->enable();
}

void LinkStepperMotor::calibrate(bool calibrationDirection, bool calibrationNO) {
	if (this->calibrationPin == -1) { return; }
	// Set the calibration pin to the correct mode
	if (calibrationNO) {
		pinMode(this->calibrationPin, INPUT_PULLUP);
	} else {
		pinMode(this->calibrationPin, INPUT);
	}
	// Set the calibration pin to the correct state
	calibrationDirection ? digitalWrite(this->dirPin, HIGH) : digitalWrite(this->dirPin, LOW);

	// Wait for the calibration pin to be pressed
	int initialCalibrationPinState = digitalRead(calibrationPin);
	while (digitalRead(calibrationPin) != initialCalibrationPinState) {
		this->stepMotor();
	}

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
	if (timeSinceLastStep > this->currentDelay) {
		// Swap HIGH/LOW pulse on every occurance of the delay
		this->isRunning = !this->isRunning;
		this->isRunning ? digitalWrite(this->stepPin, HIGH) : digitalWrite(this->stepPin, LOW);
		this->previousModulationTime = currentTime;
		// Speed can be reassigned here to compute a new currentDelay depending on the progress of the movement
		if (!this->isRunning) {
			// Increment/Decrement steps during LOW pulse in order to avoid missing steps
			if (this->currentPosition != this->targetPosition) {
				this->currentPosition < this->targetPosition ? this->currentPosition++ : this->currentPosition--;
			}
		}
	}
}

void LinkStepperMotor::stepMotor() {
	digitalWrite(this->stepPin, HIGH);
	delayMicroseconds(this->currentDelay);
	digitalWrite(this->stepPin, LOW);
	delayMicroseconds(this->currentDelay);
}

bool LinkStepperMotor::isMoving() { return this->currentPosition != this->targetPosition; }

void LinkStepperMotor::updateCurrentAngle() {
	this->currentAngle = (this->currentPosition / ((float)this->stepsPerRevolution * this->gearRatio)) * 360.0f;
}

void LinkStepperMotor::setTargetPosition(long targetPosition) {
	this->previousPosition = this->currentPosition;
	this->targetPosition = targetPosition;
	this->currentDirection = this->targetPosition > this->currentPosition;
}

void LinkStepperMotor::setTargetPositionDegrees(float targetAngleDegrees) {
	long targetPosition = this->convertDegreesToSteps(targetAngleDegrees);
	this->setTargetPosition(targetPosition);
}

void LinkStepperMotor::setSpeedRPM(float speedRPM) {
	uint16_t linkSPS = (speedRPM / 60.0f) * this->stepsPerRevolution * this->gearRatio;
	this->setSpeedSPS(linkSPS);
}

void LinkStepperMotor::setSpeedSPS(uint16_t speedSPS) {
	this->currentSpeedSPS = speedSPS;
	this->currentDelay = this->getDelayFromSpeed(speedSPS);
}

void LinkStepperMotor::setDirection(bool CW) {
	CW ? digitalWrite(this->dirPin, HIGH) : digitalWrite(this->dirPin, LOW);
	this->currentDirection = CW;
}
