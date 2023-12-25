#include "main.h"
#include "atlaslib.h"
#include <fstream>

using namespace okapi;
using namespace pros;

okapi::Motor wings(WINGS, true, okapi::AbstractMotor::gearset::green,
				   okapi::AbstractMotor::encoderUnits::counts);

okapi::Motor puncher(PUNCHER, true, okapi::AbstractMotor::gearset::green,
					 okapi::AbstractMotor::encoderUnits::counts);

okapi::Controller masterController;
okapi::ControllerButton wingsOut(okapi::ControllerDigital::R1);
okapi::ControllerButton wingsIn(okapi::ControllerDigital::R2);
okapi::ControllerButton puncherToggle(okapi::ControllerDigital::L1);
okapi::ControllerButton puncherSingleFire(okapi::ControllerDigital::L2);

bool puncherToggled = false;
bool chassisWingsForward = true;

// chassis

auto chassis = okapi::ChassisControllerBuilder()
				   .withMotors({LEFT_MTR2, LEFT_MTR1, LEFT_MTR3}, {-RIGHT_MTR2, -RIGHT_MTR1, -RIGHT_MTR3})
				   .withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 16_in}, imev5GreenTPR})
				   .withOdometry()
				   .buildOdometry();

enum class autonSelect
{
	off,
	opSide,
	allySide,
	skills
};

autonSelect autonSelection = autonSelect::off;

static const char *btnmMap[] = {"opSide", "allySide", ""};

static lv_res_t autonBtnmAction(lv_obj_t *btnm, const char *txt)
{
	if (lv_obj_get_free_num(btnm) == 100)
	{ // reds
		if (txt == "opSide")
			autonSelection = autonSelect::opSide;
		else if (txt == "allySide")
			autonSelection = autonSelect::allySide;
	}

	masterController.rumble("..");
	return LV_RES_OK; // return OK because the button matrix is not deleted
}

static lv_res_t skillsBtnAction(lv_obj_t *btn)
{
	masterController.rumble("..");
	autonSelection = autonSelect::skills;
	return LV_RES_OK;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{

	// lvgl theme
	lv_theme_t *th = lv_theme_alien_init(360, NULL); // Set a HUE value and keep font default RED
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
	std::cout << pros::millis() << ": lift: " << wings.getTemperature() << std::endl;
	std::cout << pros::millis() << ": tray: " << puncher.getTemperature() << std::endl;
}

void stopAll()
{
	chassis->stop();
	wings.moveVoltage(0);
	puncher.moveVoltage(0);
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

	switch (autonSelection)
	{
	case autonSelect::opSide:
		// opponent goalside auton
		chassis->setState({0_ft, 0_ft, 0_deg});

		// diagram.red & digram.green lines: push the ball that starts infront of the
		// robot to the goal -> go back and hit the ball with more force into the goal
		chassis->driveToPoint({-2_ft, 1.7_ft});
		chassis->driveToPoint({-2_ft, 1_ft});
		chassis->driveToPoint({-2_ft, 2_ft});
		chassis->turnToPoint({-2_ft, 2_ft});

		// set chassis state to -2,2
		chassis->setState({-2_ft, 2_ft, 0_deg});
		// this is already where the program thinks the robot is
		// this is for when im testing

		// diagram.blue & diagram.white lines: move back -> rotate -> open wings
		chassis->driveToPoint({-2_ft, 1.2_ft});
		chassis->turnAngle(90_deg);
		wings.moveAbsolute(1000, 200);
		pros::delay(500);

		// diagram.lightBlue & diagram.white lines: move forward to push the
		// matchload ball and retract the wings while moving backwards
		chassis->driveToPoint({0_ft, .8_ft});
		wings.moveAbsolute(0, 200);
		chassis->driveToPoint({-.2_ft, .8_ft});

		// diagram.pink line: push the matchload ball and the one under the
		// hang bar to my zone
		chassis->driveToPoint({1.4_ft, .5_ft});
		// opponent goalside auton end
		break;

	case autonSelect::allySide:
		// ally goalside auton
		chassis->setState({0_ft, 0_ft, 0_deg});

		// push ball infornt into goal
		chassis->driveToPoint({0_ft, 6_ft});

		// rotate and extend wings
		// extend wings
		wings.moveAbsolute(1000, 200);
		pros::delay(500);

		chassis->turnAngle(90_deg);
		// ally goalside auton end

	default:
		break;
	}

	// Stop all motors
	stopAll();
}

void opcontrol()
{
	chassis->stop();
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis->setMaxVelocity(200);
	float joystickAvg = 0;

	while (true)
	{
		if (chassisWingsForward)
		{
			auto chassis = okapi::ChassisControllerBuilder()
							   .withMotors({LEFT_MTR2, LEFT_MTR1, LEFT_MTR3}, {-RIGHT_MTR2, -RIGHT_MTR1, -RIGHT_MTR3})
							   .withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 16_in}, imev5GreenTPR})
							   .withOdometry()
							   .buildOdometry();
		}
		else if (!chassisWingsForward)
		{
			auto chassis = okapi::ChassisControllerBuilder()
							   .withMotors({-LEFT_MTR2, -LEFT_MTR1, -LEFT_MTR3}, {RIGHT_MTR2, RIGHT_MTR1, RIGHT_MTR3})
							   .withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 16_in}, imev5GreenTPR})
							   .withOdometry()
							   .buildOdometry();
		}
		chassis->getModel()->tank(masterController.getAnalog(okapi::ControllerAnalog::leftY),
								  masterController.getAnalog(okapi::ControllerAnalog::rightY));
		// chassis is set to coast mode -> motors dont forcefully stop, they coast

		// if wingsout is pressed then move the wings and keep the wings on hold, else wings motor is set to 0 and
		// coasts to a stop
		if (wingsOut.isPressed())
		{
			wings.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			wings.moveVoltage(12000);
		}
		else if (wingsIn.isPressed())
		{
			wings.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			wings.moveVoltage(-12000);
		}
		else
		{
			wings.moveVoltage(0);
		}

		// single fire pucnher button setup
		if (puncherSingleFire.isPressed())
		{
			puncherToggled = false; // Single fire button is pressed, reset toggle state/untoggle puncher
			puncher.moveVoltage(12000);
		}
		else if (puncherSingleFire.changedToReleased())
		{
			puncher.moveVoltage(0);
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
			puncher.moveVoltage(0);
		}

		if (puncherToggled)
		{
			puncher.moveVoltage(12000);
		}
		else if (!puncherToggled && !puncherSingleFire.isPressed())
		{
			puncher.moveVoltage(0);
		}

		pros::delay(20);
	}
}