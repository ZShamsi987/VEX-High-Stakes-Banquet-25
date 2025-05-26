#include "main.h" // this now includes <vector> and <cmath> for us, and api.h

// define your drive motors and imu sensor here
// !! make sure these ports match your robot's physical wiring !!
// example motor group definitions:
// motor port 1 is normal, motor port 2 is reversed, motor port 3 is normal for the left side
pros::MotorGroup left_mg({1, -2, 3}); 
// motor port 4 is reversed, motor port 5 is normal, motor port 6 is reversed for the right side
pros::MotorGroup right_mg({-4, 5, -6}); 
// imu sensor on port 10 (adjust as needed)
pros::Imu imu(10); 

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// IMPORTANT: calibrate the imu
	// this takes a few seconds, robot must be completely still
	pros::lcd::set_text(3, "IMU calibrating...");
	imu.reset(); // this call blocks until calibration is complete

	// a short loop to ensure calibration is done and give user feedback
	// imu.reset() is blocking, but is_calibrating() can be used if reset is non-blocking in a future update
	// or if you initiated calibration asynchronously (which imu.reset() does not)
	// for now, the blocking imu.reset() is sufficient.
	// while (imu.is_calibrating()) { // this loop isn't strictly needed after a blocking imu.reset()
	// 	pros::lcd::set_text(4, "Calibrating... wait");
	// 	pros::delay(50); 
	// }
	pros::lcd::set_text(3, "IMU calibrated! Ready.");
	pros::delay(1000); // show message for a second
	// pros::lcd::clear_line(3); // optional: clear the calibration message after a bit
    // pros::lcd::clear_line(4);

	// if you were to implement the odometry task from odometry.hpp, you'd start it here.
	// example (if odom_task_runner was defined and odometry.hpp uncommented):
	// pros::Task odom_tracker_task(odom_task_runner_function_pointer, nullptr, "Odom Task");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    pros::lcd::set_text(2, "Robot Disabled.");
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
void competition_initialize() {
    pros::lcd::set_text(2, "Competition Init.");
    // example: put your auton selector code here
}

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
void autonomous() {
	pros::lcd::set_text(2, "Auton Mode Running...");
	
	// call your auton function from autons.cpp
	// you'll need to uncomment the functions in autons.cpp and this call
	// run_selected_auton(); 

	// for direct testing of your PID functions (make sure they are uncommented in autons.cpp):
	// extern void drive_pid(double target_degrees, int max_speed_mv); // if not in main.h already
	// extern void turn_pid(double target_heading_degrees, int max_speed_mv);
	// drive_pid(720, 9000); // example: drive forward (needs tuning!)
	// pros::delay(500);
	// turn_pid(90.0, 7000); // example: turn to 90 degrees (needs tuning!)
    pros::lcd::set_text(3, "Auton placeholder finished.");
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	// motor groups are already defined globally, so no need to redefine them here

	pros::lcd::set_text(2, "OpControl Running!");

	while (true) {
		// lcd print status of buttons
		pros::lcd::print(0, "Buttons: L:%d C:%d R:%d", 
                         (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// simple arcade control scheme
		int forward_mv = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn_mv = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		
        // apply to motor groups (move takes values from -127 to 127, move_voltage from -12000 to 12000)
        // for direct joystick to motor percent:
        // left_mg.move(forward_mv - turn_mv);
        // right_mg.move(forward_mv + turn_mv);

        // if you want to scale joystick (0-127) to voltage (-12000 to 12000)
        // roughly 127 maps to 12000mV, so multiply by ~94.5
        // (12000 / 127 = 94.48)
        double scale_factor = 12000.0 / 127.0;
		left_mg.move_voltage((forward_mv - turn_mv) * scale_factor);
		right_mg.move_voltage((forward_mv + turn_mv) * scale_factor);

		// display current imu heading for fun
		// pros::lcd::print(4, "IMU Heading: %.2f", imu.get_heading());


		pros::delay(20); // Run for 20 ms then update, ~50Hz
	}
}