#include "main.h"
#include "atlaslib.h"
#include <fstream>


okapi::Motor wings(WINGS, true, okapi::AbstractMotor::gearset::green,
							okapi::AbstractMotor::encoderUnits::counts);

okapi::Motor puncher(PUNCHER, true, okapi::AbstractMotor::gearset::green,
							okapi::AbstractMotor::encoderUnits::counts);

okapi::Controller masterController;
okapi::ControllerButton wingsOut(okapi::ControllerDigital::R1);
okapi::ControllerButton wingsIn(okapi::ControllerDigital::R2);
okapi::ControllerButton puncherToggle(okapi::ControllerDigital::L1);
okapi::ControllerButton puncherSingleFire(okapi::ControllerDigital::L2);

// chassis
auto chassis = okapi::ChassisControllerBuilder()
				   .withMotors({LEFT_MTR2, LEFT_MTR1, LEFT_MTR3}, {-RIGHT_MTR2, -RIGHT_MTR1, -RIGHT_MTR3})
				   .withDimensions(okapi::AbstractMotor::gearset::green, (60.0 / 36.0), {{3.25_in, 17_in}, okapi::imev5GreenTPR})
				   // do withdimensions okapi with a green gearset 3.25 in wheels and 16 wheel track
				   .withOdometry()

				   .buildOdometry();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	
}

void stop()
{
	chassis.stop();
	wings.stop();
	puncher.stop();
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
void autonomous()
{
	// Set up motor objects

	// push ball infornt into goal
	// move for seconds
	

	// rotate and extend wings
	

	// Stop all motors
	stop();
}

void opcontrol()
{
	chassis->stop();
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis->setMaxVelocity(200);
	float joystickAvg = 0;

	while (true)
	{
		chassis->getModel()->tank(masterController.getAnalog(okapi::ControllerAnalog::leftY),
								masterController.getAnalog(okapi::ControllerAnalog::rightY));

		//if wingsout is pressed then move the wings ,else wings motor is set to 0
		if (wingsOut.isPressed())
		{
			wings.moveVoltage(12000);
		}
		else if (wingsIn.isPressed())
		{
			wings.moveVoltage(-12000);
		}
		else
		{
			wings.moveVoltage(0);
		}

		pros::delay(20);
	}
}