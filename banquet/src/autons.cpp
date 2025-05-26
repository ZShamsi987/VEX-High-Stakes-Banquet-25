#include "main.h"        
#include "libpid/pid.hpp" 
// #include "libpid/odometry.hpp" // if we were using full odom, but not for these examples

// these need to be defined in main.cpp (or somewhere global) and declared extern here
// example: in main.cpp: pros::MotorGroup left_mg...;
//          in autons.cpp: extern pros::MotorGroup left_mg;
extern pros::MotorGroup left_mg; //BS names change after
extern pros::MotorGroup right_mg;
extern pros::Imu imu; // the imu sensor

// --- autonomous movement functions ---

/*
 * function: drive_pid
 * purpose: drive the robot forward or backward a target distance using pid
 * target_degrees: motor encoder degrees to travel. positive for forward, negative for backward.
 * max_speed_mv: maximum voltage (absolute value) to apply to motors (e.g., 9000 out of 12000)
 */
/*
void drive_pid(double target_degrees, int max_speed_mv) {
    // pid constants - 
    // start with p (e.g., 10.0-50.0 for degrees), d (e.g., 50.0-200.0), i usually 0 or very small for drive.
    double p = 20.0; 
    double i = 0.01; // small i might help with final push, but can cause issues
    double d = 70.0;

    libpid::PIDController drive_controller(p, i, d, -max_speed_mv, max_speed_mv);

    // reset motor encoders and pid state
    left_mg.tare_position();
    right_mg.tare_position();
    drive_controller.reset();

    bool settled = false;
    int settle_counter = 0;
    const int settle_threshold_loops = 15; // needs to be stable for ~15 loops * 20ms = 300ms
    const double error_tolerance_degrees = 15.0; // how close is "good enough" in degrees

    int timeout_counter = 0;
    const int timeout_loops = 250; // 250 loops * 20ms = 5 seconds timeout

    pros::lcd::set_text(4, "Drive PID running");

    while (!settled && timeout_counter < timeout_loops) {
        // 1. get current average motor position
        // get_positions() returns a std::vector<double>
        double left_pos = left_mg.get_positions()[0]; 
        double right_pos = right_mg.get_positions()[0];
        double average_current_pos = (left_pos + right_pos) / 2.0;

        // 2. calculate pid output
        double motor_power = drive_controller.calculate(target_degrees, average_current_pos);

        // 3. apply power to motors
        left_mg.move_voltage(motor_power);
        right_mg.move_voltage(motor_power);

        // 4. check for settlement
        double current_error = target_degrees - average_current_pos;
        if (std::abs(current_error) < error_tolerance_degrees) {
            settle_counter++;
        } else {
            settle_counter = 0; // reset if we move out of tolerance
        }

        if (settle_counter >= settle_threshold_loops) {
            settled = true;
        }

        timeout_counter++;
        pros::delay(20); // loop delay, ~50hz
    }

    // stop motors after movement or timeout
    left_mg.move_voltage(0);
    right_mg.move_voltage(0);

    if (settled) {
        pros::lcd::set_text(4, "Drive PID: Settled");
    } else {
        pros::lcd::set_text(4, "Drive PID: Timeout");
    }
    pros::delay(200); // quick pause to see message
}
*/

/*
 * function: turn_pid
 * purpose: turn the robot to a target absolute heading using imu and pid
 * target_heading_degrees: absolute heading (0-360).
 * max_speed_mv: maximum voltage for turning motors.
 */
/*
void turn_pid(double target_heading_degrees, int max_speed_mv) {
    // pid constants for turning - also need tuning!
    // p for turn might be higher (e.g., 80-200), d also significant (e.g., 200-800). i often 0.
    double p_turn = 100.0; 
    double i_turn = 0.5;  // small i for turn can help stick the landing
    double d_turn = 350.0;

    libpid::PIDController turn_controller(p_turn, i_turn, d_turn, -max_speed_mv, max_speed_mv);
    turn_controller.reset();

    // imu should be calibrated and providing continuous heading (0-360)

    bool settled = false;
    int settle_counter = 0;
    const int settle_threshold_loops_turn = 15; 
    const double error_tolerance_degrees_turn = 1.5; // tighter tolerance for turns

    int timeout_counter = 0;
    const int timeout_loops_turn = 150; // 150 loops * 20ms = 3 seconds timeout

    pros::lcd::set_text(5, "Turn PID running");

    while (!settled && timeout_counter < timeout_loops_turn) {
        double current_heading = imu.get_heading();

        // calculate shortest path error (e.g., target 10, current 350 -> error should be +20, not -340)
        double error = target_heading_degrees - current_heading;
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }
        // now 'error' is the shortest angle to turn, from -180 to 180

        // pid controller aims to make current_value == setpoint.
        // we want our 'error' (shortest path) to become 0.
        // so, setpoint for pid is 0, current_value is -error (or current_value is error, setpoint is 0, if error is defined as current-target)
        // With error = target - current, we want to drive this error to 0.
        // So the PID's target is 0, and its current value is effectively our calculated 'error'.
        // Let PID calculate (0 - (-error)) = error.
        double motor_power = turn_controller.calculate(0, -error);


        // apply power to motors (opposite for turning)
        left_mg.move_voltage(motor_power);
        right_mg.move_voltage(-motor_power);

        // check for settlement (using the shortest path error)
        if (std::abs(error) < error_tolerance_degrees_turn) {
            settle_counter++;
        } else {
            settle_counter = 0;
        }

        if (settle_counter >= settle_threshold_loops_turn) {
            settled = true;
        }

        timeout_counter++;
        pros::delay(20);
    }

    // stop motors
    left_mg.move_voltage(0);
    right_mg.move_voltage(0);
    
    if (settled) {
        pros::lcd::set_text(5, "Turn PID: Settled");
    } else {
        pros::lcd.set_text(5, "Turn PID: Timeout");
    }
    pros::delay(200);
}
*/

// --- your actual autonomous routine(s) ---
/*
void my_sample_auton() {
    pros::lcd::set_text(3, "Sample Auton Start");

    // example sequence:
    drive_pid(720, 9000); // drive forward ~2 rotations (720 deg), max 9000mV
    pros::delay(200);     // lil pause

    turn_pid(90.0, 6000); // turn to 90 degrees absolute, max 6000mV
    pros::delay(200);

    drive_pid(-360, 8000); // drive backward ~1 rotation
    pros::delay(200);
    
    turn_pid(0.0, 6000);   // turn back to 0 degrees (face forward)

    pros::lcd::set_text(3, "Sample Auton Done!");
}
*/

// this function can be called from autonomous() in main.cpp
void run_selected_auton() {
    // here you could have a selector or just run one auton
    // my_sample_auton();
}