#include "main.h"
#include "atlaslib.h"
#include <fstream>

pros::Motor leftmotor_1(1);
pros::Motor leftmotor_2(2);
pros::Motor leftmotor_3(3);
pros::Motor rightmotor_1(8);
pros::Motor rightmotor_2(9);
pros::Motor rightmotor_3(10);
pros::Motor wingflaps(6);
pros::Motor_Group leftmtrs({leftmotor_1, leftmotor_2, leftmotor_3});
pros::Motor_Group rightmtrs({rightmotor_1, rightmotor_2, rightmotor_3});
pros::Controller master(CONTROLLER_MASTER);

// chassis
auto chassis = okapi::ChassisControllerBuilder()
				   .withMotors({LEFT_MTR2, LEFT_MTR1, LEFT_MTR3}, {-RIGHT_MTR2, -RIGHT_MTR1, -RIGHT_MTR3})



				   
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
	leftmotor_1.move_velocity(0);
	leftmotor_2.move_velocity(0);
	leftmotor_3.move_velocity(0);
	rightmotor_1.move_velocity(0);
	rightmotor_2.move_velocity(0);
	rightmotor_3.move_velocity(0);
	wingflaps.move_velocity(0);
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
	leftmtrs.move(127);
	rightmtrs.move(-127);
	pros::delay(675);
	leftmtrs.move(0);
	rightmtrs.move(0);

	// rotate and extend wings
	wingflaps.move(-127);
	pros::delay(1200);
	wingflaps.move(0);
	leftmtrs.move(-127);
	rightmtrs.move(-127);
	pros::delay(420);
	leftmtrs.move(0);
	rightmtrs.move(0);

	// Stop all motors
	stop();
}

void opcontrol()
{

	while (true)
	{

		// arcade drive [1 stick]

		// int power = master.get_analog(ANALOG_LEFT_Y);
		// int turn = master.get_analog(ANALOG_LEFT_X);
		// int left = power + turn;
		// int right = power - turn;
		// leftmotor_group.move(-left);
		// rightmotor_group.move(right);

		// tank drive [2 sticks]

		int left = master.get_analog(ANALOG_RIGHT_Y);
		int right = master.get_analog(ANALOG_LEFT_Y);
		leftmtrs.move(left);
		rightmtrs.move(-right);

		if (master.get_digital(DIGITAL_R1))
		{
			wingflaps.move(127);
		}
		else if (master.get_digital(DIGITAL_R2))
		{
			wingflaps.move(-127);
		}
		else
		{
			wingflaps.move(0);
		}

		pros::delay(20);
	}
}