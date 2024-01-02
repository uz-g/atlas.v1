#include "main.h"
#include "atlaslib.h"
#include <fstream>
#include "okapi/api.hpp"

using namespace okapi;
using namespace pros;
using namespace std;

// create the motors
okapi::Motor wings(WINGS, false, okapi::AbstractMotor::gearset::green,
				   okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor puncher(PUNCHER, true, okapi::AbstractMotor::gearset::red,
					 okapi::AbstractMotor::encoderUnits::degrees);

// create controller
okapi::Controller masterController;
okapi::ControllerButton wingsIn(okapi::ControllerDigital::R1);
okapi::ControllerButton wingsOut(okapi::ControllerDigital::R2);
okapi::ControllerButton wingsToggle(okapi::ControllerDigital::B);
okapi::ControllerButton puncherToggle(okapi::ControllerDigital::L1);
okapi::ControllerButton puncherSingleFire(okapi::ControllerDigital::L2);
okapi::ControllerButton reverseButton(okapi::ControllerDigital::X);

bool puncherToggled = false; // for the puncher toggle button, allows for
							 // the puncher to be toggled on and off
bool reverseFlag = false;	 // for the chassis wings front button, allows for
							 // the driver controls to be reversed

// chassis

// create the chassis object/motors with the correct wheels and gearset
auto chassis = okapi::ChassisControllerBuilder()
				   .withMotors({LEFT_MTR1, LEFT_MTR2, LEFT_MTR3}, {-RIGHT_MTR1, -RIGHT_MTR2, -RIGHT_MTR3})
				   .withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 16_in}, imev5GreenTPR})
				   .withOdometry()
				   .buildOdometry();

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

enum class wingsState
{
	RETRACTED,
	RETRACTING,
	EXTENDED,
	EXTENDING
};

wingsState wingsCurrentStatus = wingsState::RETRACTED;

// activate wings function designed for minimal gear slip and motor burnout
void toggleWings()
{
	// get the state of wings:

	if (wings.getActualVelocity() > 10) // wings are extending
	{
		wingsCurrentStatus = wingsState::RETRACTING;
		printf("wings state: retracting\n");
	}
	else if (wings.getActualVelocity() < -10) // wings are retracting
	{
		wingsCurrentStatus = wingsState::EXTENDING;
		printf("wings state: extending\n");
	}
	else if (wings.getPosition() < -360) // wings are extended already
	{
		wingsCurrentStatus = wingsState::EXTENDED;
		printf("wings state: extended\n");
	}
	else if (wings.getPosition() > -360) // wings are retracted already
	{
		wingsCurrentStatus = wingsState::RETRACTED;
		printf("wings state: retracted\n");
	}
	

	//	act based on the state of wings 
	printf("%f", wings.getPosition());

	switch (wingsCurrentStatus)
	{
	case wingsState::RETRACTED:
		wings.moveAbsolute(-1120, 200);
		printf("wings action: was retracted -> is now extending\n");
		break;

	case wingsState::RETRACTING:
		wings.moveAbsolute(-1120, 200);
		printf("wings action: was retracting -> is now extending\n");
		break;

	case wingsState::EXTENDED:
		wings.moveAbsolute(0, 200);
		printf("wings action: was extended -> is now retracting\n");
		break;

	case wingsState::EXTENDING:
		wings.moveAbsolute(0, 200);
		printf("wings action: was extending -> is now retracting\n");
		break;
	}
}

void punchForAmount(int amount)
{
	puncher.moveRelative(amount, 100);
}

void restartChassis()
{
	// chassis is set to coast mode -> motors dont forcefully stop, they coast
	chassis->stop();
	wings.moveVelocity(0);
	puncher.moveVelocity(0);	  // stop everything on the robot
	chassis->setMaxVelocity(200); // max velocity of 200 just in case
	reverseFlag = false;		  // forward on joysticks -> wings are in the front
}

void stopAll() // stops everything
{
	chassis->stop();
	wings.moveVelocity(0);
	puncher.moveVelocity(0);
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
	lv_mbox_set_text(msgBox, "rick from r");
	lv_obj_align(msgBox, NULL, LV_ALIGN_CENTER, 0, 20);
	lv_mbox_set_anim_time(msgBox, 300);
	lv_mbox_start_auto_close(msgBox, 2000);

	std::cout << pros::millis() << ": finished creating gui!" << std::endl;

	// log motor temps
	std::cout << pros::millis() << "\n"
			  << pros::millis() << ": motor temps:" << std::endl;
	std::cout << pros::millis() << ": wings: " << wings.getTemperature() << std::endl;
	std::cout << pros::millis() << ": puncher: " << puncher.getTemperature() << std::endl;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled()
{
	stopAll();
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
	restartChassis();
	chassis->setMaxVelocity(200);
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	puncher.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	wings.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	auto timer = TimeUtilFactory().create().getTimer();
	timer->placeMark();

	if (autonSelection == autonState::off)
		autonSelection = autonState::opSide; // use opside [the better side for us] if we havent selected an auton

	switch (autonSelection)
	{
	case autonState::opSide:
		// opponent goalside auton
		chassis->setState({2_ft, 0_ft, 0_deg});

		// diagram.red & digram.green lines: push the ball that starts infront of the
		// robot to the goal -> go back and hit the ball with more force into the goal
		chassis->driveToPoint({.4_ft, 1.7_ft});
		chassis->driveToPoint({0_ft, 1_ft});
		chassis->driveToPoint({0_ft, 2_ft});

		chassis->setState({0_ft, 2_ft, 0_deg}); // change this to where ever the robot ends up while testin

		// diagram.blue & diagram.white lines: move back -> rotate -> open wings
		chassis->driveToPoint({1.2_ft, .9_ft});
		chassis->turnToAngle(90_deg);
		toggleWings();

		// diagram.lightBlue & diagram.white lines: move forward to push the
		// matchload ball and retract the wings while moving backwards
		chassis->driveToPoint({2_ft, 1_ft});
		wings.moveAbsolute(0, 200);
		chassis->driveToPoint({1.8_ft, 1_ft});

		// diagram.pink line: push the matchload ball and the one under the
		// hang bar to my zone
		chassis->driveToPoint({4_ft, 1_ft});
		// opponent goalside auton end
		break;

	case autonState::allySide:
		// ally goalside auton
		chassis->setState({8_ft, 0_ft, 0_deg});

		// take the ball out of the matchload zone [no code for this yet]
		chassis->driveToPoint({9_ft, 1_ft});
		chassis->driveToPoint({9.5_ft, 1_ft}); // drive next to matchload zone

		chassis->turnToAngle(90_deg); // rotate
		toggleWings();
		// open wings

		chassis->driveToPoint({7_ft, 1_ft}); // go back to take the ball out of the matchload zone

		chassis->setState({7_ft, 1_ft, 0_deg}); // change this to where ever the robot ends up while testing
		toggleWings();
		// close wings

		// push ball infornt into goal
		chassis->driveToPoint({8_ft, 4_ft});
		chassis->driveToPoint({7_ft, 4.8_ft});

		// rotate and extend wings
		// extend wings
		toggleWings();

		chassis->turnToAngle(90_deg);
		// ally goalside auton end

	case autonState::skills:
		chassis->setState({2_ft, 0_ft, 0_deg});

		// diagram.red & digram.green lines: push the ball that starts infront of the
		// robot to the goal -> go back and hit the ball with more force into the goal
		chassis->driveToPoint({0_ft, 1.7_ft});
		chassis->driveToPoint({0_ft, 1_ft});
		chassis->driveToPoint({0_ft, 2_ft});

		// set chassis state
		chassis->setState({0_ft, 2_ft, 0_deg}); // change this to where ever the robot ends up while testing

		// move to matchload pipe and turn to face offensive zone
		chassis->driveToPoint({1_ft, 1_ft});
		chassis->turnAngle(45_deg);

		// punch for 35 seconds
		punchForAmount(25);

		// drive up to the middle pipe and turn to face the goal
		chassis->driveToPoint({2.3_ft, 5_ft});
		chassis->turnToAngle(-90_deg);

		// activate wings and drive forward to push the ball into the offensive zone
		toggleWings();

		chassis->driveToPoint({9_ft, 5_ft});
		toggleWings();

		chassis->setState({9_ft, 5_ft, 0_deg}); // change this to where ever the robot ends up while testing

		// go to right side of goal and push balls in
		chassis->driveToPoint({8_ft, .5_ft});
		chassis->driveToPoint({9.5_ft, 2_ft});
		chassis->setState({9.5_ft, 2_ft, 0_deg}); // change this to where ever the robot ends up while testing

		// go to left side of goal and push balls in
		chassis->driveToPoint({7_ft, 2_ft});
		chassis->driveToPoint({8_ft, 7.7_ft});
		chassis->driveToPoint({10_ft, 8_ft});

	case autonState::testing:
		chassis->setState({2_ft, 0_ft, 0_deg});
		toggleWings();
		// will be testing the rotations and the threshold for this function

	default:
		break;
	}
	std::cout << pros::millis() << ": auton took " << timer->getDtFromMark().convert(second) << " seconds" << std::endl;

	// Stop all motors
	restartChassis();
}

void opcontrol()
{
	restartChassis();
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	puncher.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	wings.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	while (true)
	{
		if (reverseButton.changedToPressed()) // reverse flag toggle if button is pressed
		{
			reverseFlag = !reverseFlag;
		}

		// Get joystick values so that they can be manipulated for reversal
		double leftJoystick = masterController.getAnalog(okapi::ControllerAnalog::leftY);
		double rightJoystick = masterController.getAnalog(okapi::ControllerAnalog::rightY);

		// Reverse controls if needed
		if (reverseFlag)
		{
			leftJoystick = -leftJoystick;
			rightJoystick = -rightJoystick;
		}

		// Pass the manipulated joystick values to tank drive thing
		chassis->getModel()->tank(leftJoystick, rightJoystick);

		// if wingsout is pressed then move the wings and keep the wings on hold, else wings motor is set to 0 and
		// coasts to a stop

		if (wingsToggle.changedToPressed())
		{
			printf("\n wings toggle button is pressed");
			toggleWings();
			// toggle wings is pushed
		}

		if (wingsIn.isPressed())
		{
			wings.moveVelocity(200);
			// r1 is pushed
		}
		else if (wingsOut.isPressed())
		{
			wings.moveVelocity(-200);
			// r2 is pushed
		}
		else if (wingsOut.changedToReleased() || wingsIn.changedToReleased())
		{
			wings.moveVelocity(0);
			// printf("\n wings are set to 0 velocity");
		}

		// single fire pucnher button setup
		if (puncherSingleFire.isPressed())
		{
			puncherToggled = false; // Single fire button is pressed, reset toggle state/untoggle puncher
			puncher.moveVelocity(100);
		}
		else if (puncherSingleFire.changedToReleased())
		{
			puncher.moveVelocity(0);
		}

		// if puncherToggle is pressed then keep the puncher motor on max speed and keep the puncher on coast,
		// else puncher motor is set to 0 and coasts to a stop

		if (puncherToggle.isPressed() && !puncherToggled)
		{
			puncherToggled = true; // Button was just pressed, toggle the state
		}
		else if (puncherToggle.isPressed() && puncherToggled)
		{
			puncherToggled = false; // Button is released, reset toggle state
			puncher.moveVelocity(0);
		}

		if (puncherToggled)
		{
			puncher.moveVelocity(100);
		}

		if (!puncherToggled && !puncherSingleFire.isPressed())
		{
			puncher.moveVelocity(0);
		}

		//printf( "\n wings position: %f"  , wings.getPosition());
		pros::delay(20);
	}
}