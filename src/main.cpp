#include "main.h"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
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
    pros::Motor leftmotor_1(1);
    pros::Motor leftmotor_2(2);
    pros::Motor leftmotor_3(3);
    pros::Motor rightmotor_1(8);
    pros::Motor rightmotor_2(9);
    pros::Motor rightmotor_3(10);
    pros::Motor wingflaps(6);
	pros::Motor_Group leftmotor_group({leftmotor_1, leftmotor_2, leftmotor_3});
	pros::Motor_Group rightmotor_group({rightmotor_1, rightmotor_2, rightmotor_3});

	//push ball infornt into goal
	//move for seconds
	leftmotor_group.move(127);
	rightmotor_group.move(-127);
	pros::delay(550);
	leftmotor_group.move(0); 
    rightmotor_group.move(0); 

	//rotate and extend wings
	wingflaps.move(-127);
	pros::delay(1500);
	wingflaps.move(0);
	leftmotor_group.move(127);
	rightmotor_group.move(127);
	pros::delay(420);
	leftmotor_group.move(0); 
    rightmotor_group.move(0); 









	

    // // Move for seconds
    // leftmotor_group.move_velocity(127); // Adjust velocity as needed
    // rightmotor_group.move_velocity(-127); // Adjust velocity as needed
    // pros::delay(180);
	// leftmotor_group.move_velocity(0); // Adjust velocity as needed
    // rightmotor_group.move_velocity(0); // Adjust velocity as needed
	

    // // Rotate left 90 degrees
    // leftmotor_group.move_velocity(-127); // Adjust velocity as needed
    // rightmotor_group.move_velocity(-127); // Adjust velocity as needed
    // pros::delay(415); // Adjust delay for the desired turn angle
	// leftmotor_group.move_velocity(0); // Adjust velocity as needed
    // rightmotor_group.move_velocity(0);

	// //move a bit
	// leftmotor_group.move_velocity(100); // Adjust velocity as needed
	// rightmotor_group.move_velocity(-100); // Adjust velocity as needed
	// pros::delay(520);
	// leftmotor_group.move_velocity(0); // Adjust velocity as needed
    // rightmotor_group.move_velocity(0);

	// //wing flaps extend
	
	// wingflaps.move_velocity(-127);
	// pros::delay(1600);
	// leftmotor_group.move_velocity(0); // Adjust velocity as needed
    // rightmotor_group.move_velocity(0);

	// //move a bit
	// leftmotor_group.move_velocity(-127); // Adjust velocity as needed
	// rightmotor_group.move_velocity(127); // Adjust velocity as needed
	// pros::delay(680);

  
    // Stop all motors
    leftmotor_1.move_velocity(0);
    leftmotor_2.move_velocity(0);
    leftmotor_3.move_velocity(0);
    rightmotor_1.move_velocity(0);
    rightmotor_2.move_velocity(0);
    rightmotor_3.move_velocity(0);
    wingflaps.move_velocity(0);
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
void opcontrol()
{
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor right2_mtr(1);
	// pros::Motor right_mtr(2);
	// pros::Motor left_mtr(19);
	// pros::Motor left2_mtr(20);

	// while (true) {
	// 	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
	// 	int left = master.get_analog(ANALOG_LEFT_Y);
	// 	int right = master.get_analog(ANALOG_RIGHT_Y);

	// 	right2_mtr = right;
	// 	right_mtr = right;
	// 	left_mtr = -left;
	// 	left2_mtr = -left;

	// 	pros::delay(20);

	// arcade drive

	pros::Motor leftmotor_1(1);
	pros::Motor leftmotor_2(2);
	pros::Motor leftmotor_3(3);
	pros::Motor rightmotor_1(8);
	pros::Motor rightmotor_2(9);
	pros::Motor rightmotor_3(10);
	pros::Motor puncher(5);
	pros::Motor wingflaps(6);
	
	pros::Motor_Group leftmotor_group({leftmotor_1, leftmotor_2, leftmotor_3});
	pros::Motor_Group rightmotor_group({rightmotor_1, rightmotor_2, rightmotor_3});

	pros::Controller master(CONTROLLER_MASTER);

	while (true)
	{

		//arcade drive [1 stick]


		// int power = master.get_analog(ANALOG_LEFT_Y);
		// int turn = master.get_analog(ANALOG_LEFT_X);
		// int left = power + turn;
		// int right = power - turn;
		// leftmotor_group.move(-left);
		// rightmotor_group.move(right);


		//tank drive [2 sticks]

		int left = master.get_analog(ANALOG_RIGHT_Y);
		int right = master.get_analog(ANALOG_LEFT_Y);
		leftmotor_group.move(left);
		rightmotor_group.move(-right);

		if(master.get_digital(DIGITAL_R1))
		{
			wingflaps.move(127);
		}
		else if(master.get_digital(DIGITAL_R2))
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
