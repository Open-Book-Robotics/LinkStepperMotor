#ifndef LINKSTEPPERMOTOR_HPP
#define LINKSTEPPERMOTOR_HPP
/**
 * @file LinkStepperMotor.hpp
 * @author Sharwin Patil, OpenBook Robotics
 * @brief A Stepper Motor library for Arduino with a specific focus on robotic arm applications.
 * @version 2.0
 * @date 2023-11-09
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
	LinkStepperMotor(const LinkStepperMotor&) = default;

	/**
	 * @brief Assignment operator deleted to prevent copying of object.
	 *
	 */
	LinkStepperMotor& operator=(const LinkStepperMotor&) = default;

	/**
	 * @brief Parameterized constructor to initialize the LinkStepperMotor object.
	 *
	 * @param stepPin The pin number to which the step pin of the stepper motor is connected.
	 * @param dirPin The pin number to which the direction pin of the stepper motor is connected.
	 * @param enablePin The pin number to which the enable pin of the stepper motor is connected.
	 * @param calibrationPin The pin number to which a calibration sensor (limit switch or hall effect) of the stepper motor is connected.
	 * @param stepsPerRevolution The number of steps per revolution of the stepper motor, ensure that this value already accounts for microstepping.
	 * @param gearRatio The gear ratio of the output. (Eg. 1:5 gear ratio would be 5, for 5 revolutions of the motor, the output would rotate 1 revolution)
	 * @param minAngle The minimum angle of the motor [degrees]
	 * @param maxAngle The maximum angle of the motor [degrees]
	 *
	 * @note If the motor does not have a calibration sensor, set this value to -1.
	 */
	LinkStepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t calibrationPin, uint16_t stepsPerRevolution, float gearRatio, float minAngle, float maxAngle) :
		stepPin(stepPin), dirPin(dirPin), enablePin(enablePin), calibrationPin(calibrationPin),
		stepsPerRevolution(stepsPerRevolution), gearRatio(gearRatio), minAngle(minAngle), maxAngle(maxAngle) {}

	/**
	 * @brief Enables the motor by setting the enable pin to LOW.
	 *
	 */
	void enable();

	/**
	 * @brief Disables the motor by setting the enable pin to HIGH.
	 *
	 */
	void disable();

	/**
	 * @brief Reads the enable pin to determine if the motor is enabled or not.
	 *
	 * @return true if the motor is enabled
	 */
	bool isEnabled();

	/**
	 * @brief Initializes the motor by setting the pin modes, initial pin states and enables the motor
	 *
	 */
	void initialize();

	/**
	 * @brief Calibrates the motor by moving it in the specified direction until the calibration pin differs from its original state.
	 *
	 * @param calibrationDirection the direction in which to calibrate the motor [True -> CW, False -> CCW]
	 * @param calibrationNO true if the calibration pin is normally open, false if the calibration pin is normally closed, defaults to true
	 *
	 */
	void calibrate(bool calibrationDirection, bool calibrationNO = true);

	/**
	 * @brief Gets the current motor position [steps]
	 *
	 * @return int16_t representing the current position of the motor [steps]
	 */
	int16_t getCurrentPosition();

	/**
	 * @brief Gets the target motor position [steps]
	 *
	 * @return int16_t representing the target position of the motor [steps]
	 */
	int16_t getTargetPosition();

	/**
	 * @brief Gets the current absolute angle of the motor [degrees] by converting the current position [steps] to degrees
	 *
	 * @return float representing the current angle of the motor [degrees]
	 */
	float getCurrentAngle();

	/**
	 * @brief Gets the current speed of the motor [steps/second]
	 *
	 * @return uint16_t representing the current speed of the motor [steps/second]
	 */
	uint16_t getSpeedSPS();

	/**
	 * @brief Gets the current pulse interval [microseconds] between steps
	 *
	 * @note This value is used as the delay between HIGH pulses to the step pin to control the speed of the motor.
	 *
	 * @return unsigned long representing the current pulse interval [microseconds]
	 */
	unsigned long getPulseInterval();

	/**
	 * @brief Determines if the given angle [degrees] is in the range of motion of the motor.
	 *
	 * @param angleDegrees The angle to check [degrees]
	 * @return true if the angle is within the range of motion of the motor
	 */
	bool isInRangeOfMotion(float angleDegrees);

	/**
	 * @brief Sets the target motor position [steps] given the desired absolute angle [degrees]
	 *
	 * @param targetAngleDegrees The desired absolute angle of the motor [degrees]
	 */
	void setTargetPositionDegrees(float targetAngleDegrees);

	/**
	 * @brief Sets the motor speed to the given speed [RPM]
	 *
	 * @note The motor speed is controlled by the pulse interval [microseconds], so this function
	 * 			 handles the conversion from [RPM] to [steps/second] to [microseconds] internally.
	 *
	 * @param speedRPM The desired speed of the motor [RPM]
	 */
	void setSpeedRPM(float speedRPM);

	/**
	 * @brief Determines if the motor is moving or not using the current and target positions.
	 *
	 * @return true if the motor is moving
	 */
	bool isMoving();

	/**
	 * @brief Asynchronous update function that utilizes the current pulse interval and a micros() timer
	 * 				to determine when to step the motor.
	 *
	 */
	void update();

	/**
	 * @brief Asynchronous update function that utilizes an acceleration profile to determine the current pulse interval
	 * 				at each step and a micros() timer to determine when to step the motor.
	 *
	 */
	void updateAccel();

	/**
	 * @brief Performs a single step using the current pulse interval.
	 *
	 */
	void stepMotor();

private:
	const uint8_t stepPin;
	const uint8_t dirPin;
	const uint8_t enablePin;
	const uint8_t calibrationPin;
	const uint16_t stepsPerRevolution;
	const float gearRatio;
	const float minAngle;
	const float maxAngle;

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
	volatile int16_t currentPosition = 0;

	/**
	 * @brief The previous assigned target position of the motor [steps]
	 *
	 */
	int16_t previousTargetPosition = 0;

	/**
	 * @brief The target position of the motor [steps]
	 *
	 * @note This value is updated by setting the target but only update() functions will move the motor.
	 */
	int16_t targetPosition = 0;

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
	volatile unsigned long previousModulationTime = 0;

	/**
	 * @brief The current pu;se interval between steps [microseconds]
	 *
	 * @note This is computed using the current speed and acceleration.
	 */
	unsigned long currentPulseInterval = 0;

	/**
	 * @brief The current speed of the motor [steps/second]
	 *
	 * @note Defaults to 60 RPM [1 revolution/second] which is the steps per revolution of the motor.
	 *
	 */
	uint16_t currentSpeedSPS = this->stepsPerRevolution;

	/**
	 * @brief The current speed of the motor [revolutions/minute]
	 *
	 * @note Defaults to 60 RPM
	 *
	 */
	float currentSpeedRPM = 60;

	/**
	 * @brief The initial speed of the motor when performing motion-profiled movement [steps/second]
	 *
	 */
	static constexpr int16_t initialSpeedSPS = 8000;

	/**
	 * @brief The k (acceleration rate) value used in the acceleration profile [steps/second^2]
	 *
	 */
	static constexpr int16_t accelerationRate = 200;

	/**
	 * @brief The initial acceleration of the motor when performing motion-profiled movement [steps/second^2]
	 *
	 */
	static constexpr int16_t initialAcceleration = 200;

	/**
	 * @brief Sets the target motor position [steps]
	 *
	 * @param targetPosition The desired target position of the motor [steps]
	 */
	void setTargetPosition(int16_t targetPosition);

	/**
	 * @brief Sets the direction of the motor [True -> CW, False -> CCW]
	 *
	 * @note This function writes to the digital pin to set the direction in addition to updating the internal field.
	 *
	 * @param CW The desired direction of the motor [True -> CW, False -> CCW]
	 */
	void setDirection(bool CW);

	/**
	 * @brief Sets the current motor speed [steps/second]
	 *
	 * @note This function internally computes the pulse interval [microseconds] and updates the internal field.
	 *
	 * @param speedSPS The desired speed of the motor [steps/second]
	 */
	void setSpeedSPS(uint16_t speedSPS);

	/**
	 * @brief Converts the target output angle into steps the link motor needs to accomplish that angle
	 *
	 * @note This is NOT relative to the current position of the motor, it is simply a conversion function of desired output degrees to steps
	 *
	 * @param degrees The target angle of the motor [degrees] [0, 360)
	 * @return int16_t The number of steps the motor needs to move to accomplish the target angle
	 */
	int16_t convertDegreesToSteps(float degrees) const { return round((degrees / 360.0f) * this->stepsPerRevolution * this->gearRatio); }

	/**
	 * @brief Given a speed [steps/second], calculate the pulse interval between steps [microseconds]
	 *
	 * @param speed The speed of the motor [steps/second]
	 *
	 * @return unsigned long representing the pulse interval between steps [microseconds]
	 */
	static unsigned long getPulseIntervalFromSpeed(uint16_t speed) { return (unsigned long)(1000000.0f / speed); }

	/**
	 * @brief Given a pulse interval [microseconds], calculate the speed [steps/second]
	 *
	 * @param pulseInterval The pulse interval between steps [microseconds] which determines the stepper motor speed
	 * @return uint16_t representing the speed of the motor [steps/second]
	 */
	static uint16_t getSpeedFromPulseInterval(unsigned long pulseInterval) { return (uint16_t)(1000000.0f / pulseInterval); }

	/**
	 * @brief Computes the current angle [degrees] using the current motor position [steps]
	 * 				and updates the internal field.
	 *
	 */
	void updateCurrentAngle();

	void computeNewPulseInterval();
};

#endif // LINKSTEPPERMOTOR_HPP