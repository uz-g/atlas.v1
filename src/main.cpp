#include "atlaslib.h"

using namespace okapi;
using namespace std;

// create the motors
Motor intakeExp(INTAKE_EXP, false, okapi::AbstractMotor::gearset::green,
				okapi::AbstractMotor::encoderUnits::degrees); // exp vex motor 5.5w
Motor cata(CATAPULT, false, okapi::AbstractMotor::gearset::green,
		   okapi::AbstractMotor::encoderUnits::degrees); // normal motor
Motor cataExp(CATAPULT_EXP, false, okapi::AbstractMotor::gearset::green,
			  okapi::AbstractMotor::encoderUnits::degrees); // exp vex motor 5.5w

MotorGroup catapult({cata, cataExp});

// create controller + buttons
Controller masterController;
ControllerButton reverseButton(okapi::ControllerDigital::X);
ControllerButton intake(okapi::ControllerDigital::R1);
ControllerButton outtake(okapi::ControllerDigital::R2);
ControllerButton catapultToggle(okapi::ControllerDigital::down);
ControllerButton catapultHold(okapi::ControllerDigital::L1);
ControllerButton cataHang(okapi::ControllerDigital::L2);
ControllerButton cataDescore(okapi::ControllerDigital::A);

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
					   {-LEFT_MTR_B, -LEFT_MTR_M, -LEFT_MTR_F},
					   {RIGHT_MTR_B, RIGHT_MTR_M, RIGHT_MTR_F}) // left motor is reversed
				   .withGains(									// the i in pid is usually not needed for vex pids so keep it 0
					   {0.00000, 0, 0.00000},					// distance controller gains (p, i, d)
					   {0.00000, 0, 0.00000},					// turn controller gains (p, i, d)
					   {0.00000, 0, 0.00000}					// angle controller gains (helps drive straight) (p, i, d)
					   )
				   // dema filters were here, no longer here because i dont know how to use them
				   // dual exponential moving average filters - smooths out the controller output and make it less jerky
				   .withDimensions(
					   {AbstractMotor::gearset::green, (36.0 / 60.0)}, // green motor cartridge, 36:60 gear ratio
					   {{3.25_in, 14.75_in}, imev5GreenTPR})		   // 3.25 inch wheels, 14.75 inch wheelbase width

				   .withMaxVelocity(200)
				   .withSensors(
					   RotationSensor{xRotationSensor},		 // vertical encoder in V5 port 1
					   RotationSensor{yRotationSensor, true} // horizontal encoder in V5 port 2 (reversed)
					   )
				   .withOdometry(
					   {{2.75_in},
						quadEncoderTPR},
					   StateMode::CARTESIAN)
				   // 2.75 inch wheels, 7 inch wheelbase width, and tpr for v5 rotation sensor
				   // 1 horizontal tracking wheel and 1 vertical tracking wheel not sure how to do that

				   .buildOdometry();

auto profileController = AsyncMotionProfileControllerBuilder()
							 .withLimits( // base values * modifier values
								 {
									 1.439 * .8, // max velocity in m/s calculated using squiggles constraints * a modifier that i want
									 // max acceletation
									 // these motor can go to 1.05 nm of torque, will use .9 nm of torque to be safe
									 //  estimatd weight of 15 kg
									 // 8.7219866748 m/s/s max acceleration, ill use 4 [i found this on a forum maybe its a good idea to
									 //  keep it low] and apply a modifier of .8 - 2 may also be a good default value
									 3.00 * .8,
									 8.00 * .8 // max jerk should be around double max acceleration
								 })			   // double maxVel double maxAccel double maxJerk
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
			masterController.rumble("__");

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
										   "	Cata: %d\n"
										   "IntakeExp: %d"
										   "	CataExp: %d\n"
										   "left back: %d"
										   "	left middle: %d\n"
										   "left front: %d"
										   "	right back: %d\n"
										   "right middle: %d"
										   "	right front: %d\n",
			 cata.getTemperature(), intakeExp.getTemperature(), cataExp.getTemperature(),
			 clbm.getTemperature(), clmm.getTemperature(), clfm.getTemperature(),
			 crbm.getTemperature(), crmm.getTemperature(), crfm.getTemperature());

	lv_label_set_text(motorTemps, tempsText);
	lv_obj_set_pos(motorTemps, 0, 0);
	lv_obj_align(motorTemps, NULL, LV_ALIGN_CENTER, 0, 0);

	std::cout << pros::millis() << ": finished creating gui!" << std::endl;

	// log motor temps
	std::cout << pros::millis() << "\n"
			  << pros::millis() << ": motor temps:" << std::endl;
	std::cout << pros::millis() << ": cata: " << cata.getTemperature() << std::endl;
	std::cout << pros::millis() << ": intakeExp: " << intakeExp.getTemperature() << std::endl;
	std::cout << pros::millis() << ": cataExp: " << cataExp.getTemperature() << std::endl;
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
	catapult.moveVelocity(0);
	intakeExp.moveVelocity(0);
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
	disabled(); // stop all motors
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	intakeExp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
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
			{{0_in, 31_in, 0_deg}, {30_in, 0_in, 0_deg}, {26_in, 0_in, 0_deg}, {30_in, 0_in, 0_deg}}, "A");
		// Make the chassis follow the path
		profileController->setTarget("A");
		profileController->waitUntilSettled();

		// Generate a path that descores the matchload ball
		profileController->generatePath(
			{{30_in, 0_in, 0_deg}, {20_in, 4_in, 90_deg}, {10_in, 14_in, 90_deg}, {20_in, 71_in, 90_deg}}, "B");

		catapult.moveRelative(720, 200);

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
			catapult.moveVelocity(200);
		}
		catapult.moveVelocity(0);

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

	disabled(); // stop all motors
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void opcontrol()
{
	disabled();
	chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	intakeExp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	masterController.rumble("..-..-..");

	bool reversed = false;

	bool catapultOn = false;

	bool liftIsToggled = false;

	bool cataHanging = false;

	bool cataDescoring = false;

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

		// Manually handle joystick controls and reverse if necessary
		double leftJoystick = reversed ? -leftJoystick : leftJoystick;
		double rightJoystick = reversed ? -rightJoystick : rightJoystick;
		chassis->getModel()->tank(leftJoystick, rightJoystick);

		// Pass the manipulated joystick values to tank drive thing

		// cata controls and stuff
		if (catapultHold.isPressed())
		{
			catapult.moveVelocity(200);
		}
		else if (catapultToggle.changedToPressed())
		{
			catapultOn = !catapultOn;
		}
		else if (cataHang.changedToPressed())
		{
			cataHanging = !cataHanging;
		}
		else if (cataDescore.isPressed())
		{
			cataDescoring = !cataDescoring;
		}
		else if (catapultOn)
		{
			catapult.moveVelocity(200);
		}
		else if (cataHanging)
		{
			catapult.moveRelative(720, 200);
		}
		else if (cataDescoring)
		{
			catapult.moveRelative(720, 200);
		}
		else if (catapultOn)
		{
			catapult.moveVelocity(600);
		}
		else
		{
			catapult.moveVelocity(0);
		}

		// intake controls and stuff
		if (intake.isPressed())
		{
			intakeExp.moveVelocity(200);
		}
		else if (outtake.isPressed())
		{
			intakeExp.moveVelocity(-200);
		}
		else
		{
			intakeExp.moveVelocity(0);
		}

		pros::delay(20); // delay that is required for pros to work
						 // default delay of 20ms but i can try 10ms later to see if its better
	}
}