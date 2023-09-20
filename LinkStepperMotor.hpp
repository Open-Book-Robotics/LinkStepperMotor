#pragma once
/**
 * @file LinkStepperMotor.hpp
 * @author Sharwin Patil, RoboKits
 * @brief A Stepper Motor library for Arduino with a specific focus on robotic arm applications.
 * @version 1.0
 * @date 2023-08-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>


 /// TODO: Documentation

/**
* @brief A Stepper Motor library for Arduino with a specific focus on robotic arm applications.
*
*/
class LinkStepperMotor {
public:

	/**
	 * @brief Deleted default constructor to enforce use of parameterized constructor.
	 *
	 */
	LinkStepperMotor() = delete;

	/**
	 * @brief Default destructor.
	 *
	 */
	~LinkStepperMotor() = default;

	/**
	 * @brief Copy constructor deleted to prevent copying of object.
	 *
	 */
	LinkStepperMotor(const LinkStepperMotor&) = delete;

	/**
	 * @brief Assignment operator deleted to prevent copying of object.
	 *
	 */
	LinkStepperMotor& operator=(const LinkStepperMotor&) = delete;

	/**
	 * @brief Parameterized constructor to initialize the LinkStepperMotor object.
	 *
	 * @param stepPin The pin number to which the step pin of the stepper motor is connected.
	 * @param dirPin The pin number to which the direction pin of the stepper motor is connected.
	 * @param enablePin The pin number to which the enable pin of the stepper motor is connected.
	 * @param calibrationPin The pin number to which a calibration sensor (limit switch or hall effect) of the stepper motor is connected.
	 * @param stepsPerRevolution The number of steps per revolution of the stepper motor, ensure that this value already accounts for microstepping.
	 * @param gearRatio The gear ratio of the output. (Eg. 1:5 gear ratio would be 5, for 5 revolutions of the motor, the output would rotate 1 revolution)
	 *
	 * @note If the motor does not have a calibration sensor, set this value to -1.
	 */
	LinkStepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t calibrationPin, uint16_t stepsPerRevolution, float gearRatio) :
		stepPin(stepPin), dirPin(dirPin), enablePin(enablePin), calibrationPin(calibrationPin), stepsPerRevolution(stepsPerRevolution), gearRatio(gearRatio) {}

	inline void enable() { digitalWrite(this->enablePin, LOW); }
	inline void disable() { digitalWrite(this->enablePin, HIGH); }
	inline bool isEnabled() const { return !digitalRead(this->enablePin); }

	void initialize();

	void calibrate(bool calibrationDirection, bool calibrationNO = true);

	inline long getCurrentPosition() const { return this->currentPosition; }
	inline long getTargetPosition() const { return this->targetPosition; }
	inline float getCurrentAngle() const { return this->currentAngle; }

	inline void setTargetPositionDegrees(float targetAngleDegrees);
	inline void setSpeedRPM(float speedRPM);

	inline bool isMoving() const;

	void update();

private:
	const uint8_t stepPin;
	const uint8_t dirPin;
	const uint8_t enablePin;
	const uint8_t calibrationPin;
	const uint16_t stepsPerRevolution;
	const float gearRatio;

	/**
	 * @brief A flag that the update() function uses to swap between a HIGH/LOW pulse on every iteration
	 *
	 */
	volatile bool isRunning = false;

	/**
	 * @brief The current angle of the motor [degrees], computed using the current position, steps per revolution and gear ratio.
	 *
	 */
	float currentAngle = 0;

	/**
	 * @brief The current position of the motor [steps]
	 *
	 */
	long currentPosition = 0;

	/**
	 * @brief The previous position of the motor [steps]
	 *
	 */
	long previousPosition = 0;

	/**
	 * @brief The target position of the motor [steps]
	 *
	 * @note This value is updated by setting the target but only update() functions will move the motor.
	 */
	long targetPosition = 0;

	/**
	 * @brief The current direction of the motor [True -> CW, False -> CCW]
	 *
	 */
	bool currentDirection = false;

	/**
	 * @brief The previous time at which the motor was stepped [microseconds]
	 *
	 * @note This is intended to be assigned to the output of micros() function.
	 */
	unsigned long previousModulationTime = 0;

	/**
	 * @brief The current delay between steps [microseconds]
	 *
	 * @note This is computed using the current speed and acceleration.
	 */
	unsigned long currentDelay = 0;

	/**
	 * @brief The current speed of the motor [steps/second]
	 *
	 * @note Defaults to 60 RPM (1 revolution/second)
	 *
	 */
	uint16_t currentSpeedSPS = this->stepsPerRevolution * this->gearRatio;

	/**
	 * @brief The current speed of the motor [revolutions/minute]
	 *
	 * @note Defaults to 60 RPM
	 *
	 */
	float currentSpeedRPM = 60;

	/**
	 * @brief The current acceleration of the motor [steps/second^2]
	 *
	 */
	uint16_t currentAcceleration = 0;

	inline void setTargetPosition(long targetPosition);
	inline void setDirection(bool CW);
	inline void setSpeedSPS(uint16_t speedSPS);

	/**
	 * @brief Converts the target output angle into steps the link motor needs to accomplish that angle
	 *
	 * @note This is NOT relative to the current position of the motor, it is simply a conversion function of desired output degrees to steps
	 *
	 * @param degrees The target angle of the motor [degrees] [0, 360)
	 * @return uint16_t The number of steps the motor needs to move to accomplish the target angle
	 */
	inline uint16_t convertDegreesToSteps(float degrees) const { return round((degrees / 360.0f) * this->stepsPerRevolution * this->gearRatio); }

	/**
	 * @brief Given a speed [steps/second], calculate the delay between steps [microseconds]
	 *
	 * @param speed The speed of the motor [steps/second]
	 *
	 * @return unsigned long representing the delay between steps [microseconds]
	 */
	static inline unsigned long getDelayFromSpeed(uint16_t speed) { return (long)(1000000.0f / speed); }

	/**
	 * @brief Performs a single step using the current delay.
	 *
	 */
	void stepMotor();

	inline void updateCurrentAngle();
};