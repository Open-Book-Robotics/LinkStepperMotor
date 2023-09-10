#include "LinkStepperMotor.hpp"

constexpr uint16_t STEPS_PER_REV = 200 * 16; // 1/16th Microstepping

int main() {
	LinkStepperMotor motor = LinkStepperMotor(1, 2, 3, 4, STEPS_PER_REV, 5.0);
	std::cout << "Constructed" << std::endl;
	return 0;
}