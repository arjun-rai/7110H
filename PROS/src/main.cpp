#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
Controller controller;
std::shared_ptr<ChassisController> drive = ChassisControllerBuilder()
	.withMotors(
		{7,2},
		{-6, -14}
	)
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10_in}, imev5GreenTPR})
	.build();
std::shared_ptr<AsyncMotionProfileController> profileController = 
	AsyncMotionProfileControllerBuilder()
	.withLimits({
		1.0,
		2.0,
		10.0
	})
	.withOutput(drive)
	.buildMotionProfileController();
void autonomous() {
	profileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 0_deg}}, "A");
	profileController->setTarget("A");
	profileController->waitUntilSettled();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                              controller.getAnalog(ControllerAnalog::rightX));
		pros::delay(20);
	}
}
