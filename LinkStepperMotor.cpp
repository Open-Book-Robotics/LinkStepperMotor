#include "LinkStepperMotor.hpp"
#include <Arduino.h>

void LinkStepperMotor::enable() {
	this->isEnabled = true;
	digitalWrite(this->enablePin, LOW);
}

void LinkStepperMotor::disable() {
	this->isEnabled = false;
	digitalWrite(this->enablePin, HIGH);
}

void LinkStepperMotor::initialize(float linkRPM) {
	// Setup pins
	pinMode(this->stepPin, OUTPUT);
	pinMode(this->dirPin, OUTPUT);

	// Convert RPM to steps/second
	uint16_t linkSPS = (linkRPM / 60.0f) * this->stepsPerRevolution * this->gearRatio;
	this->setSpeed(linkSPS);
	this->enable();
}

void LinkStepperMotor::calibrate(bool calibrationDirection, uint8_t calibrationPin, bool calibrationNO) {
	// Set the calibration pin to the correct mode
	if (calibrationNO) {
		pinMode(calibrationPin, INPUT_PULLUP);
	} else {
		pinMode(calibrationPin, INPUT);
	}
	this->calibrationPin = calibrationPin;
	// Set the calibration pin to the correct state
	if (calibrationDirection) {
		digitalWrite(this->dirPin, HIGH);
	} else {
		digitalWrite(this->dirPin, LOW);
	}

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
	// If the motor is not enabled, do nothing and if the motor is not moving, we have reached the target
	if (!this->isEnabled || !this->isMoving) { return; }

	// Determine the time since the last step
	unsigned long currentTime = micros();
	long timeSinceLastStep = currentTime - this->previousModulationTime;
	if (timeSinceLastStep > this->currentDelay) {
		// Swap HIGH/LOW pulse on every occurance of the delay
		this->isRunning = !this->isRunning;
		this->isRunning ? digitalWrite(this->stepPin, HIGH) : digitalWrite(this->stepPin, LOW);
		this->previousModulationTime = currentTime;
		// Speed can be reassigned here to compute a new currentDelay depending on the progress of the movement
		if (!this->isRunning) {
			// Increment/Decrement steps during LOW pulse in order to avoid missing steps
			this->currentPosition < this->targetPosition ? this->currentPosition++ : this->currentPosition--;
		}
	}
}

void LinkStepperMotor::stepMotor() {
	digitalWrite(this->stepPin, HIGH);
	delayMicroseconds(this->currentDelay);
	digitalWrite(this->stepPin, LOW);
	delayMicroseconds(this->currentDelay);
}

void LinkStepperMotor::updateCurrentAngle() {
	// Convert current position from steps to angle in degrees
	this->currentAngle = (this->currentPosition / ((float)this->stepsPerRevolution * this->gearRatio)) * 360.0f;
}