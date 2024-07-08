#ifndef CLOSEDLOOPLINKSTEPPERMOTOR_HPP
#define CLOSEDLOOPLINKSTEPPERMOTOR_HPP
/**
 * @file ClosedLoopLinkStepperMotor.hpp
 * @author Sharwin Patil, OpenBook Robotics
 * @brief An extension of the LinkStepperMotor class that implements closed-loop control.
 * @version 1.0
 * @date 2024-07-05
 * @details Utilizes the AS5600 magnetic encoder to implement closed-loop control.
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <Arduino.h>
#include <AS5600.h>

 /// TODO: Implement AS5600 Encoder using library to be used as a LinkMotor
 /// TODO: Extend the stepper motor class to include closed-loop methods or to override movement methods with CL

class Encoder {
public:
	Encoder() = default;
	~Encoder() = default;

	bool init(void);

	float getEncoderPosition(void);
private:
	const static AS5600 encoder = AS5600();
	float currentEncoderPosition = 0.0;
};

class ClosedLoopLinkStepperMotor : public LinkStepperMotor {
public:
	ClosedLoopLinkStepperMotor() = delete;
	~ClosedLoopLinkStepperMotor() = default;

	ClosedLoopLinkStepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, int8_t calibrationPin, uint16_t stepsPerRevolution, float gearRatio, float minAngle, float maxAngle)


private:
	const Encoder encoder = Encoder();
};

#endif