#include "main.h"
#include "atlaslib.h"
#include <fstream>
#include "okapi/api.hpp"

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

// the driver controls to be reversed

// chassis

// create the chassis object/motors with the correct wheels and gearset
auto chassis = ChassisControllerBuilder()
				   .withMotors(
					   {-LEFT_MTR1, -LEFT_MTR2, -LEFT_MTR3},
					   {RIGHT_MTR1, RIGHT_MTR2, RIGHT_MTR3}) // left motor is reversed
				   .withGains(
					   {kPDist, kIDist, kDDist},   // distance controller gains (p, i, d)
					   {kPTurn, kITurn, kDTurn},   // turn controller gains (p, i, d)
					   {kPAngle, kIAngle, kDAngle} // angle controller gains (helps drive straight) (p, i, d)
					   )
				   .withSensors(
					   RotationSensor{xRotationSensor},		 // Left encoder in V5 port 1
					   RotationSensor{yRotationSensor, true} // Right encoder in V5 port 2 (reversed)
					   )
				   .withDimensions(
					   {AbstractMotor::gearset::green, (60.0 / 36.0)}, // green motor cartridge, 60:36 gear ratio
					   {{3.25_in, 12_in}, imev5GreenTPR})			   // 3.25 inch wheels, 14.75 inch wheelbase width

				   .withOdometry({{2.75_in, 7_in}, quadEncoderTPR}) // 2.75 inch wheels, 7 inch wheelbase width
				   .buildOdometry();

// create chassis controller pid

auto profileController = AsyncMotionProfileControllerBuilder()
							 .withLimits({1.06 * .9,
										  2.00 * .9,
										  10.00 * .9}) // double maxVel double maxAccel double maxJerk
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
	auto timer = TimeUtilFactory().create().getTimer();
	timer->placeMark();

	auto flywheelTimer = TimeUtilFactory().create().getTimer();
	flywheelTimer->placeMark();

	if (autonSelection == autonState::off)
		autonSelection = autonState::opSide; // use opside [the better side for us] if we havent selected an auton

	switch (autonSelection)
	{
	case autonState::opSide:
		// opponent goalside auton
		chassis->setState({0_ft, 24_in, 0_deg});
		profileController->generatePath(
			{{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "A");
		// Make the chassis follow the path
		profileController->setTarget("A");
		profileController->waitUntilSettled();

		break;

	case autonState::allySide:
		// ally goalside auton
		chassis->setState({8_ft, 0_ft, 0_deg});

		break;
	case autonState::skills:
		chassis->setState({24_in, 0_ft, 0_deg});

		chassis->driveToPoint({.4_ft, 3.5_ft});

		// move to matchload pipe and turn to face offensive zone
		chassis->driveToPoint({1_ft, 1.1_ft});
		chassis->turnToPoint({9_ft, 5_ft});


		// Move the flywheel for the specified duration
		while (flywheelTimer->getDtFromMark().convert(second) < 30.0)
		{
			flywheel.moveVelocity(600);
		}
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