#include "main.h"
#include "atlaslib.h"
#include <fstream>
#include "okapi/api.hpp"
#include <iostream>

using namespace okapi;
using namespace std;

// create the motors
okapi::Motor lift(LIFT, false, okapi::AbstractMotor::gearset::red,
				  okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor flywheel(FLYWHEEL, false, okapi::AbstractMotor::gearset::blue,
					  okapi::AbstractMotor::encoderUnits::degrees);

// create controller
okapi::Controller masterController;
okapi::ControllerButton reverseButton(okapi::ControllerDigital::X);
okapi::ControllerButton flywheelToggle(okapi::ControllerDigital::R1);
okapi::ControllerButton flywheelForward(okapi::ControllerDigital::R2);
okapi::ControllerButton flywheelBackward(okapi::ControllerDigital::down);
okapi::ControllerButton liftUpManual(okapi::ControllerDigital::L1);
okapi::ControllerButton liftDownManual(okapi::ControllerDigital::L2);
okapi::ControllerButton liftToggle(okapi::ControllerDigital::A);

// chassis
// create the chassis object/motors with the correct wheels and gearset
auto chassis = ChassisControllerBuilder()
				   .withMotors(
					   {-LEFT_MTR1, -LEFT_MTR2, -LEFT_MTR3},
					   {RIGHT_MTR1, RIGHT_MTR2, RIGHT_MTR3}) // left motor is reversed
				   .withGains(
					   {dkP, dkI, dkD}, // distance controller gains (p, i, d)
					   {tkP, tkI, tkD}, // turn controller gains (p, i, d)
					   {akP, akI, akD}	// angle controller gains (helps drive straight) (p, i, d)
					   )
				   .withDerivativeFilters( // filters for the controllers makes it smoother and stuff [i have no idea]
					   std::make_unique<DemaFilter>(0.2, 0.15), // Distance controller filter
					   std::make_unique<DemaFilter>(0.2, 0.15), // Turn controller filter
					   std::make_unique<DemaFilter>(0.2, 0.15)  // Angle controller filter
					   )//dema = dual exponential moving average filters - smooths out the controller output and make it less jerky
				   .withSensors(
					   RotationSensor{xRotationSensor},		 // Left encoder in V5 port 1
					   RotationSensor{yRotationSensor, true} // Right encoder in V5 port 2 (reversed)
					   )
				   .withDimensions(
					   {AbstractMotor::gearset::green, (60.0 / 36.0)}, // green motor cartridge, 60:36 gear ratio
					   {{3.25_in, 12_in}, imev5GreenTPR})			   // 3.25 inch wheels, 14.75 inch wheelbase width

				   .withMaxVelocity(200)
				   .withOdometry(
					   {{2.75_in, 7_in},
						quadEncoderTPR}) // 2.75 inch wheels, 7 inch wheelbase width, and tpr for v5 rotation sensor
				   .buildOdometry();

auto profileController = AsyncMotionProfileControllerBuilder()
							 .withLimits( // base values * modifier values
								 {1.00 * .99,
								  2.00 * .8,
								  10.00 * .8}) // double maxVel double maxAccel double maxJerk
							 .withOutput(chassis)
							 .buildMotionProfileController();

enum class autonState
{
	off,
	opSide,
	allySide,
	skills,
	testing
};

autonState autonSelection = autonState::off;

static const char *btnmMap[] = {"opSide", "allySide", "testing", ""}; // button matrix map for auton selection

static lv_res_t autonBtnmAction(lv_obj_t *btnm, const char *txt) // button matrix action for auton selection
{
	if (lv_obj_get_free_num(btnm) == 100)
	{ // reds
		if (txt == "opSide")
			autonSelection = autonState::opSide;
		else if (txt == "allySide")
			autonSelection = autonState::allySide;
		else if (txt == "testing")
			autonSelection = autonState::testing;
	}

	masterController.rumble("..");
	return LV_RES_OK; // return OK because the button matrix is not deleted
}

static lv_res_t skillsBtnAction(lv_obj_t *btn) // button action for skills auton selection
{
	masterController.rumble("..");
	autonSelection = autonState::skills;
	return LV_RES_OK;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() // initialize the GIU
{

	// lvgl theme
	lv_theme_t *th = lv_theme_alien_init(300, NULL); // Set a HUE value and keep font default MAGENTA
	lv_theme_set_current(th);

	// create a tab view object
	std::cout << pros::millis() << ": creating gui..." << std::endl;
	lv_obj_t *tabview = lv_tabview_create(lv_scr_act(), NULL);

	// add 4 tabs (the tabs are page (lv_page) and can be scrolled
	lv_obj_t *mainTab = lv_tabview_add_tab(tabview, "Autons");
	lv_obj_t *skillsTab = lv_tabview_add_tab(tabview, "Skills");
	lv_obj_t *telemetryTab = lv_tabview_add_tab(tabview, "Telemetry");

	// main tab
	lv_obj_t *mainBtnm = lv_btnm_create(mainTab, NULL);
	lv_btnm_set_map(mainBtnm, btnmMap);
	lv_btnm_set_action(mainBtnm, autonBtnmAction);
	lv_obj_set_size(mainBtnm, 450, 50);
	lv_btnm_set_toggle(mainBtnm, true, 3);
	lv_obj_set_pos(mainBtnm, 0, 100);
	lv_obj_align(mainBtnm, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_free_num(mainBtnm, 100);

	// skills tab
	lv_obj_t *skillsBtn = lv_btn_create(skillsTab, NULL);
	lv_obj_t *label = lv_label_create(skillsBtn, NULL);
	lv_label_set_text(label, "Skills");
	lv_btn_set_action(skillsBtn, LV_BTN_ACTION_CLICK, skillsBtnAction);
	lv_obj_set_size(skillsBtn, 450, 50);
	lv_btnm_set_toggle(skillsBtn, true, 1);
	lv_obj_set_pos(skillsBtn, 0, 100);
	lv_obj_align(skillsBtn, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_free_num(skillsBtn, 102);

	// debug
	lv_obj_t *msgBox = lv_mbox_create(telemetryTab, NULL);
	lv_mbox_set_text(msgBox, "debug");
	lv_obj_align(msgBox, NULL, LV_ALIGN_CENTER, 0, 20);
	lv_mbox_set_anim_time(msgBox, 300);
	lv_mbox_start_auto_close(msgBox, 2000);

	std::cout << pros::millis() << ": finished creating gui!" << std::endl;

	// log motor temps
	std::cout << pros::millis() << "\n"
			  << pros::millis() << ": motor temps:" << std::endl;
	std::cout << pros::millis() << ": flywheel: " << flywheel.getTemperature() << std::endl;
	std::cout << pros::millis() << ": lift: " << lift.getTemperature() << std::endl;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled()
{
	chassis->stop();
	flywheel.moveVelocity(0);
	lift.moveVelocity(0);
}

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
	chassis->stop();
	flywheel.moveVelocity(0);
	lift.moveVelocity(0);
	chassis->setMaxVelocity(200);
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	lift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	std::unique_ptr<okapi::AbstractTimer> timer = TimeUtilFactory().create().getTimer();
	timer->placeMark();
	std::unique_ptr<okapi::AbstractTimer> flywheelTimer = TimeUtilFactory().create().getTimer();
	if (autonSelection == autonState::off)
		autonSelection = autonState::opSide; // use opside [the better side for us] if we havent selected an auton

	switch (autonSelection)
	{
	case autonState::opSide:
		// opponent goalside auton
		chassis->setState({0_in, 31_in, 0_deg}); // starting pos of middle of robot

		// Generate a path that hits a ball into the goal
		profileController->generatePath(
			{{0_in, 24_in, 0_deg}, {42_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}, {42_in, 0_in, 0_deg}}, "A");
		// Make the chassis follow the path
		profileController->setTarget("A");
		lift.moveAbsolute(500, 100);
		profileController->waitUntilSettled();

		// Generate a path that descores the matchload ball
		profileController->generatePath(
			{{42_in, 0_in, 0_deg}, {10_in, 14_in, 90_deg}, {20_in, 71_in, 90_deg}}, "B");
		profileController->setTarget("B");
		profileController->waitUntilSettled();

		disabled(); // stop all motors

		// this auton scores a ball into the goal, descores the
		// matchload ball, and moves the the lift bar and touches it

		break;

	case autonState::allySide:
		// ally goalside auton
		chassis->setState({0_in, 96_in, 0_deg});

		// Generate a path that hits a ball into the goal
		profileController->generatePath(
			{{0_in, 96_in, 0_deg}, {42_in, 118_in, 0_deg}, {36_in, 118_in, 0_deg}, {42_in, 118_in, 0_deg}}, "C");
		profileController->setTarget("C");
		profileController->waitUntilSettled();

		// Generate a path that scroes the middle ball in
		profileController->generatePath(
			{{42_in, 118_in, 0_deg}, {34_in, 100_in, 0_deg}, {76_in, 90_in, 0_deg}, {76_in, 90_in, 90_deg}, {76_in, 115_in, 90_deg}}, "D");
		profileController->setTarget("D");
		profileController->waitUntilSettled();

		// Generate a path that touches the lift bar
		profileController->generatePath(
			{{76_in, 115_in, 90_deg}, {50_in, 98_in, 180_deg}, {6_in, 96_in, 270_deg}, {4_in, 74_in, 270_deg}}, "E");
		profileController->setTarget("E");
		profileController->waitUntilSettled();

		disabled(); // stop all motors

		// this auton scores a ball into the goal, scores the middle
		// ball, and moves the the lift bar and touches it
		break;

	case autonState::skills:
		chassis->setState({0_in, 24_in, 0_deg});

		// Generate a path that hits a ball into the goal
		profileController->generatePath(
			{{0_in, 24_in, 0_deg}, {42_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}, {42_in, 0_in, 0_deg}}, "F");
		// Make the chassis follow the path
		profileController->setTarget("F");
		lift.moveAbsolute(500, 100);
		profileController->waitUntilSettled();

		// Generate a path that touches the matchload bar
		profileController->generatePath(
			{{42_in, 0_in, 0_deg}, {10_in, 14_in, 60_deg}}, "G");
		profileController->setTarget("G");
		profileController->waitUntilSettled();

		// Move the flywheel for the specified duration
		flywheelTimer->placeMark();
		while (flywheelTimer->getDtFromMark().convert(second) < 27.5)
		{
			flywheel.moveVelocity(600);
		}

		// go to offensive zone and score the balls from front
		chassis->setState({10_in, 14_in, 60_deg});
		chassis->driveToPoint({4_in, 108_in});
		chassis->driveToPoint({30_in, 108_in});
		chassis->driveToPoint({32_in, 80_in});
		chassis->driveToPoint({48_in, 120_in}); // hit 1 from close side
		chassis->driveToPoint({36_in, 108_in});
		chassis->driveToPoint({63_in, 120_in}); // hit 2 from middle
		chassis->driveToPoint({54_in, 105_in});
		chassis->driveToPoint({78_in, 120_in}); // hit 3 from far side

		// go to the far side and score the balls from far side
		chassis->driveToPoint({128_in, 96_in});
		chassis->driveToPoint({108_in, 132_in});

		// go to close side and score the balls from close side
		chassis->driveToPoint({128_in, 96_in});
		chassis->driveToPoint({12_in, 96_in});
		chassis->driveToPoint({42_in, 130_in});
		break;

	case autonState::testing:

		break;
		// will be testing the rotations and the threshold for this function

	default:
		break;
	}
	std::cout << pros::millis() << ": auton took " << timer->getDtFromMark().convert(second) << " seconds" << std::endl;

	// Stop all motors
	chassis->stop();
	flywheel.moveVelocity(0);
	lift.moveVelocity(0);
}

void opcontrol()
{
	chassis->stop();
	flywheel.moveVelocity(0);
	lift.moveVelocity(0);
	chassis->setMaxVelocity(200);
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	lift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	bool reversed = false;

	bool flywheelOn = false;

	bool liftGoingUp = false;

	bool liftIsUp = false;

	while (true)
	{

		// chassis controls and stuff

		if (reverseButton.changedToPressed()) // reverse robot controls toggle if button is pressed
		{
			reversed = !reversed;
		}

		// Get joystick values so that they can be manipulated for reversal
		double leftJoystick = masterController.getAnalog(okapi::ControllerAnalog::leftY);
		double rightJoystick = masterController.getAnalog(okapi::ControllerAnalog::rightY);

		// Reverse controls if needed
		if (reversed)
		{
			leftJoystick = -leftJoystick;
			rightJoystick = -rightJoystick;
			chassis->getModel()->tank(rightJoystick, leftJoystick);
		}
		else
		{
			chassis->getModel()->tank(leftJoystick, rightJoystick);
		}

		// Pass the manipulated joystick values to tank drive thing

		// flywheel controls and stuff
		if (flywheelForward.isPressed())
		{
			flywheel.moveVelocity(600);
		}
		else if (flywheelBackward.isPressed())
		{
			flywheel.moveVelocity(-600);
		}
		else if (flywheelToggle.changedToPressed())
		{
			flywheelOn = !flywheelOn;
		}
		else if (flywheelOn)
		{
			flywheel.moveVelocity(600);
		}
		else
		{
			flywheel.moveVelocity(0);
		}

		// lift controls and stuff
		if (liftUpManual.isPressed())
		{
			lift.moveVelocity(100);
		}
		else if (liftDownManual.isPressed())
		{
			lift.moveVelocity(-100);
		}
		else if (liftToggle.changedToPressed())
		{
			liftGoingUp = !liftGoingUp;
		}
		else if (liftGoingUp)
		{
			lift.moveAbsolute(500, 100);
			liftGoingUp = false;
			liftIsUp = true;
		}
		else if (!liftGoingUp && liftIsUp)
		{
			lift.moveAbsolute(0, 100);
			liftIsUp = false;
		}
		else
		{
			lift.moveVelocity(0);
		}

		pros::delay(20); // delay that is required for pros to work
						 // default delay of 20ms but i can try 10ms later to see if its better
	}
}