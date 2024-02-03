#include "atlaslib.h"

using namespace okapi;
using namespace std;

// create the motors
Motor intake(INTAKE, false, okapi::AbstractMotor::gearset::green,
			 okapi::AbstractMotor::encoderUnits::degrees);

Motor flywheel(FLYWHEEL, false, okapi::AbstractMotor::gearset::green,
			   okapi::AbstractMotor::encoderUnits::degrees);

// create controller + buttons
Controller masterController;
ControllerButton reverseButton(okapi::ControllerDigital::X);
ControllerButton takeIn(okapi::ControllerDigital::R1);
ControllerButton takeOut(okapi::ControllerDigital::R2);
ControllerButton wheelFly(okapi::ControllerDigital::A);
ControllerButton toggleFlywheel(okapi::ControllerDigital::Y);
ControllerButton chassisHold(okapi::ControllerDigital::down);
ControllerButton chassisCoast(okapi::ControllerDigital::right);

// chassis
Motor clbm(LEFT_MTR_B, true, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees);
Motor clmm(LEFT_MTR_M, true, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees);
Motor clfm(LEFT_MTR_F, true, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees);
Motor crbm(RIGHT_MTR_B, false, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees);
Motor crmm(RIGHT_MTR_M, false, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees);
Motor crfm(RIGHT_MTR_F, false, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees);

// create the chassis object/motors with the correct wheels and gearset
auto chassis = ChassisControllerBuilder()
				   .withMotors(
					   {LEFT_MTR_B, LEFT_MTR_M, LEFT_MTR_F},
					   {-RIGHT_MTR_B, -RIGHT_MTR_M, -RIGHT_MTR_F}) // left motor is reversed
				   .withGains(									// the i in pid is usually not needed for vex pids so keep it 0
					   {0.00000, 0.0, 0.00000},					// distance controller gains (p, i, d)
					   {0.00000, 0.0, 0.00000},					// turn controller gains (p, i, d)
					   {0.00000, 0.0, 0.00000}					// angle controller gains (helps drive straight) (p, i, d)
					   )
				   // dema filters were here, no longer here because i dont know how to use them
				   // dual exponential moving average filters - smooths out the controller output and make it less jerky
				   .withDimensions(
					   {AbstractMotor::gearset::green, (36.0 / 60.0)}, // green motor cartridge, 36:60 gear ratio
					   {{3.25_in, 14.75_in}, imev5GreenTPR})		   // 3.25 inch wheels, 14.75 inch wheelbase width

				   .withMaxVelocity(200)
				   .withSensors(
					   RotationSensor{leftRotationSensor},		 // left rotation sensor in V5 port 1
					   RotationSensor{rightRotationSensor, true} // right rotation sensor in V5 port 2 (reversed)
					   )
				   .withOdometry(
					   {{2.75_in, 0_in},	 // Wheel diameters for X and Y sensors
						quadEncoderTPR},	 // TPR values for X and Y sensors
					   StateMode::CARTESIAN) // State mode
											 // 2.75 inch wheels, 7 inch wheelbase width, and tpr for v5 rotation sensor
											 // 1 horizontal tracking wheel and 1 vertical tracking wheel not sure how to do that
				   .buildOdometry();

auto profileController = AsyncMotionProfileControllerBuilder()
							 .withLimits( // base values * modifier values
								 {
									 1.439 * .9, // max velocity in m/s calculated using squiggles constraints * a modifier that i want
									 // max acceletation
									 // these motor can go to 1.05 nm of torque, will use .9 nm of torque to be safe
									 //  estimatd weight of 15 kg
									 // 8.7219866748 m/s/s max acceleration, ill use 4 [i found this on a forum maybe its a good idea to
									 //  keep it low] and apply a modifier of .8 - 2 may also be a good default value
									 3.00 * .7, // max acceleration
									 8.00 * .8	// max jerk should be around double max acceleration
								 })				// double maxVel double maxAccel double maxJerk
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
		{
			masterController.rumble(". _");

			autonSelection = autonState::opSide;
		}
		else if (txt == "allySide")
		{
			masterController.rumble(".. _");

			autonSelection = autonState::allySide;
		}
		else if (txt == "testing")
		{
			masterController.rumble("_._");

			autonSelection = autonState::testing;
		}
	}

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

	// gui stuff from djmango korvex

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

	// motor temps telemetry tab
	lv_obj_t *motorTemps = lv_label_create(telemetryTab, NULL);
	char tempsText[256]; // Adjust the size based on your needs

	snprintf(tempsText, sizeof(tempsText), "Motor Temps:"
										   "	intake: %d\n"
										   "flywheel: %d"
										   "	intake: %d\n"
										   "left back: %d"
										   "	left middle: %d\n"
										   "left front: %d"
										   "	right back: %d\n"
										   "right middle: %d"
										   "	right front: %d\n",
			 intake.getTemperature(), flywheel.getTemperature(), intake.getTemperature(),
			 clbm.getTemperature(), clmm.getTemperature(), clfm.getTemperature(),
			 crbm.getTemperature(), crmm.getTemperature(), crfm.getTemperature());

	lv_label_set_text(motorTemps, tempsText);
	lv_obj_set_pos(motorTemps, 0, 0);
	lv_obj_align(motorTemps, NULL, LV_ALIGN_CENTER, 0, 0);

	std::cout << pros::millis() << ": finished creating gui!" << std::endl;

	// log motor temps
	std::cout << pros::millis() << "\n"
			  << pros::millis() << ": motor temps:" << std::endl;
	std::cout << pros::millis() << ": intake: " << intake.getTemperature() << std::endl;
	std::cout << pros::millis() << ": left Exp: " << flywheel.getTemperature() << std::endl;
	std::cout << pros::millis() << ": right exp: " << intake.getTemperature() << std::endl;
	std::cout << pros::millis() << ": left back: " << clbm.getTemperature() << std::endl;
	std::cout << pros::millis() << ": left middle: " << clmm.getTemperature() << std::endl;
	std::cout << pros::millis() << ": left front: " << clfm.getTemperature() << std::endl;
	std::cout << pros::millis() << ": right back: " << crbm.getTemperature() << std::endl;
	std::cout << pros::millis() << ": right middle: " << crmm.getTemperature() << std::endl;
	std::cout << pros::millis() << ": right front: " << crfm.getTemperature() << std::endl;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled()
{
	chassis->stop();
	intake.moveVelocity(0);
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
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	std::unique_ptr<okapi::AbstractTimer> timer = TimeUtilFactory().create().getTimer();
	timer->placeMark();
	std::unique_ptr<okapi::AbstractTimer> intakeTimer = TimeUtilFactory().create().getTimer();
	if (autonSelection == autonState::off)
		autonSelection = autonState::opSide; // use opside [the better side for us] if we havent selected an auton

	switch (autonSelection)
	{
	case autonState::opSide:
		// opponent goalside auton
		chassis->setState({0_in, 31_in, 0_deg}); // starting pos of middle of robot

		// Generate a path that hits a ball into the goal
		profileController->generatePath(
			{{0_in, 31_in, 0_deg}, {34_in, 0_in, 0_deg}, {26_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "A");
		// Make the chassis follow the path
		profileController->setTarget("A");
		profileController->waitUntilSettled();


		break;

	case autonState::allySide:
		// ally goalside auton
		chassis->setState({0_in, 96_in, 0_deg});

		intake.moveRelative(720, 200); // move the intake to hold the ball

		// Generate a path that hits a ball into the goal
		profileController->generatePath(
			{{0_in, 96_in, 0_deg}, {42_in, 118_in, 0_deg}, {36_in, 118_in, 0_deg}, {42_in, 118_in, 0_deg}}, "C");
		profileController->setTarget("C");
		profileController->waitUntilSettled();

		

		// this auton scores a ball into the goal, scores the middle
		// ball, and moves the the lift bar and touches it
		break;

	case autonState::skills:
		chassis->setState({0_in, 24_in, 0_deg});

		// Generate a path that hits a ball into the goal
		profileController->generatePath(
			{{0_in, 24_in, 0_deg}, {42_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}, {42_in, 0_in, 0_deg}}, "F");
		// Make the chassis follow the path
		intake.moveVelocity(200);
		profileController->setTarget("F");
		profileController->waitUntilSettled();
		intake.moveVelocity(0);

		profileController->generatePath(
			{{42_in, 0_in, 0_deg}, {10_in, 14_in, 60_deg}}, "G");
		profileController->setTarget("G");
		profileController->waitUntilSettled();
		intake.moveRelative(-120, 200); // activate the descore arm

		// Move the flywheel for the specified duration
		intakeTimer->placeMark();
		while (intakeTimer->getDtFromMark().convert(second) < 27.5)
		{
			intake.moveVelocity(200);
		}
		intake.moveVelocity(0);
		intake.moveRelative(-240, 200); // deactivate the descore arm

		// Go to offensive zone and score balls from front
		profileController->generatePath(
			{{10_in, 14_in, 60_deg}, {4_in, 108_in, 60_deg}, {30_in, 108_in, 60_deg}}, "H");
		profileController->setTarget("H");
		profileController->waitUntilSettled();

		profileController->generatePath(
			{{30_in, 108_in, 60_deg}, {32_in, 80_in, 0_deg}, {48_in, 120_in, 90_deg}}, "I"); // Hit 1 from close side
		profileController->setTarget("I");
		profileController->waitUntilSettled();

		profileController->generatePath(
			{{48_in, 120_in, 90_deg}, {36_in, 108_in, 0_deg}, {63_in, 120_in, 90_deg}}, "J"); // Hit 2 from middle
		profileController->setTarget("J");
		profileController->waitUntilSettled();

		profileController->generatePath(
			{{63_in, 120_in, 90_deg}, {54_in, 105_in, 0_deg}, {78_in, 120_in, 180_deg}}, "K"); // Hit 3 from far side
		profileController->setTarget("K");
		profileController->waitUntilSettled();

		// Go to far side and score balls from far side
		profileController->generatePath(
			{{78_in, 120_in, 180_deg}, {128_in, 96_in, 180_deg}, {108_in, 132_in, 180_deg}}, "L");
		profileController->setTarget("L");
		profileController->waitUntilSettled();

		// Go to close side and score balls from close side
		profileController->generatePath(
			{{108_in, 132_in, 180_deg}, {128_in, 96_in, 180_deg}, {12_in, 96_in, 0_deg}, {42_in, 130_in, 0_deg}}, "M");
		profileController->setTarget("M");
		profileController->waitUntilSettled();

		// chassisLem.follow("path.txt", 2000, 15); // follow the path that has to be uploaded to the brain via sd card
		break;

	case autonState::testing:

		chassis->setState({0_in, 0_in, 0_deg});
		chassis->driveToPoint({12_in, 0_in});
		
		break;
		// will be testing the rotations and the threshold for this function

	default:
		break;
	}
	std::cout << pros::millis() << ": auton took " << timer->getDtFromMark().convert(second) << " seconds" << std::endl;

	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis->stop();
}

void opcontrol()
{
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	masterController.rumble("..-.");

	bool reversed = false;

	bool flat = false;

	bool intakeIsToggled = false;

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
			chassis->getModel()->tank(leftJoystick, rightJoystick);
		}
		else
		{
			chassis->getModel()->tank(rightJoystick, leftJoystick);
		}
    
		// Pass the manipulated joystick values to tank drive thing

		// intake controls and stuff
		if (takeIn.isPressed())
		{
			intake.moveVelocity(200);
			intakeIsToggled = false;
		}
		else if (takeOut.isPressed())
		{
			intake.moveVelocity(-50);
			intakeIsToggled = false;
		}
		else if (wheelFly.changedToPressed())
		{
			intakeIsToggled = false;

			if (flat)
			{
				intake.moveRelative(-120, 200);
			}
			else if (!flat)
			{
				intake.moveRelative(-240, 200);
			}
			flat = !flat;
		}
		else if (toggleFlywheel.changedToPressed())
		{
			intakeIsToggled = !intakeIsToggled;
		}
		else if (intakeIsToggled)
		{
			intake.moveVelocity(200);
		}
		else if (!intakeIsToggled)
		{
			intake.moveVelocity(0);
		}

		if (chassisHold.changedToPressed())
		{
			chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		}
		else if (chassisCoast.changedToPressed())
		{
			chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		}

		pros::delay(20); // delay that is required for pros to work
						 // default delay of 20ms but i can try 10ms later to see if its better
	}
}