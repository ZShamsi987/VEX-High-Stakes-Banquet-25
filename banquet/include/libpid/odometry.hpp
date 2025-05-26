#ifndef LIBPID_ODOMETRY_HPP
#define LIBPID_ODOMETRY_HPP

// this file is mostly a placeholder for future odometry
// the actual code for a working odometry system is commented out
// as per requirements, this is to show where it *would* go
// and what it *might* look like. it's "not working" right now.

// #include "pros/rtos.hpp"
// #include "pros/imu.hpp"
// #include "pros/rotation.hpp" // or motor encoders like pros::Motor::get_position()
// #include <cmath> // for M_PI, sin, cos, etc.

namespace libpid {


// this would be our struct to hold the robot's position and angle
// struct Pose {
//     double x;     // inches, or whatever unit you fancy
//     double y;     // inches
//     double theta; // radians, 'cause math likes radians
// };

// global or class member to store current pose
// Pose robot_pose = {0.0, 0.0, 0.0};

// --- configuration constants you'd need ---
// const double WHEEL_DIAMETER_INCHES = 2.75; // your wheel size, e.g., 2.75", 3.25", 4"
// const double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * M_PI;
// // for pros motor encoders (degrees): 36k for 100rpm (blue), 18k for 200rpm (green), 9k for 600rpm (red) per rotation
// // for pros::Rotation sensor: 36000 centidegrees per rotation (so 360 degrees)
// const double MOTOR_ENCODER_TICKS_PER_DEGREE = 1.0; // for integrated motor encoders in degrees
// const double ROTATION_SENSOR_TICKS_PER_DEGREE = 100.0; // centidegrees to degrees

// const double TICKS_PER_MOTOR_ROTATION = 360.0 * (18.0/36.0); // example for green cart, 360 deg * (18 tooth driven / 36 tooth driver if external gearing)
                                                            // for integrated, it's simpler: 360 deg per internal rotation, gear ratio applies
                                                            // Green cart (200rpm): 900 ticks/rev (if get_position gives V5 internal ticks not degrees)
                                                            // PROS motor.get_position() returns DEGREES. So 360 ticks per rev.

// // distance between left and right tracking wheels (center to center)
// const double TRACK_WIDTH_INCHES = 12.5; // measure this carefully!

// --- sensor pointers (you'd pass these in or have them global) ---
// extern pros::Imu imu_sensor_for_odom; // assuming you have one named like this
// extern pros::Motor left_drive_motor_for_odom; 
// extern pros::Motor right_drive_motor_for_odom;
// or
// extern pros::Rotation left_tracking_wheel;
// extern pros::Rotation right_tracking_wheel;

// --- previous sensor values (for calculating deltas) ---
// double prev_left_encoder_val = 0.0; // could be degrees or inches after conversion
// double prev_right_encoder_val = 0.0;
// double prev_imu_rad = 0.0;


// function to initialize/reset odometry (call this at start of auton)
/*
void reset_odometry_tracking(double start_x = 0.0, double start_y = 0.0, double start_theta_deg = 0.0) {
    robot_pose.x = start_x;
    robot_pose.y = start_y;
    robot_pose.theta = start_theta_deg * M_PI / 180.0;

    // important: make sure imu is calibrated 
    // imu_sensor_for_odom.set_rotation(start_theta_deg); // tell imu its current angle

    // left_drive_motor_for_odom.tare_position();
    // right_drive_motor_for_odom.tare_position();
    // or for rotation sensors:
    // left_tracking_wheel.reset_position();
    // right_tracking_wheel.reset_position();


    prev_left_encoder_val = 0.0; // store initial state after taring
    prev_right_encoder_val = 0.0;
    prev_imu_rad = robot_pose.theta; // store initial imu radians
    // pros::lcd::print(4, "Odom Reset: %.1f, %.1f, %.1f", robot_pose.x, robot_pose.y, start_theta_deg);
}
*/

// this would be the function that runs in a loop (often in its own task)
/*
void update_odometry_loop_content() {
    // 1. get current sensor values
    // double current_left_deg = left_drive_motor_for_odom.get_position();   // in degrees for motor encoders
    // double current_right_deg = right_drive_motor_for_odom.get_position(); // in degrees

    // double current_imu_deg = imu_sensor_for_odom.get_heading(); // 0-360 continuous
    // double current_imu_rad = current_imu_deg * M_PI / 180.0;

    // 2. calculate delta degrees from encoders
    // double delta_left_deg = current_left_deg - prev_left_encoder_val;
    // double delta_right_deg = current_right_deg - prev_right_encoder_val;

    // 3. convert delta degrees to inches
    // double inches_per_degree = WHEEL_CIRCUMFERENCE_INCHES / 360.0;
    // double delta_left_inches = delta_left_deg * inches_per_degree;
    // double delta_right_inches = delta_right_deg * inches_per_degree;

    // 4. calculate average distance traveled by chassis
    // double delta_dist_inches = (delta_left_inches + delta_right_inches) / 2.0;

    // 5. calculate change in orientation (delta_theta)
    // using imu is preferred for absolute theta
    // robot_pose.theta = current_imu_rad; // direct update from imu
    // double delta_theta_rad = current_imu_rad - prev_imu_rad;
    // // handle imu wrap around for delta_theta (e.g. 359 to 1 degree)
    // if (delta_theta_rad > M_PI) delta_theta_rad -= 2.0 * M_PI;
    // if (delta_theta_rad < -M_PI) delta_theta_rad += 2.0 * M_PI;

    // 6. calculate change in x and y position
    // double global_delta_x_inches, global_delta_y_inches;
    // double avg_orientation_rad = prev_imu_rad + delta_theta_rad / 2.0; // use average orientation over the tick

    // if (std::abs(delta_theta_rad) < 0.0001) { // basically straight movement
    //     global_delta_x_inches = delta_dist_inches * std::cos(avg_orientation_rad);
    //     global_delta_y_inches = delta_dist_inches * std::sin(avg_orientation_rad);
    // } else { // arc movement
    //     // more complex arc math here, or simpler model:
    //     global_delta_x_inches = delta_dist_inches * std::cos(avg_orientation_rad); // this is an approximation
    //     global_delta_y_inches = delta_dist_inches * std::sin(avg_orientation_rad); // also approximation for arc
    //     // True arc math for (x,y) is:
    //     // double turn_radius_local_coords = delta_dist_inches / delta_theta_rad;
    //     // double change_in_x_local = turn_radius_local_coords * std::sin(delta_theta_rad);
    //     // double change_in_y_local = turn_radius_local_coords * (1.0 - std::cos(delta_theta_rad)); // y is perpendicular to heading for local coords
    //     // global_delta_x_inches = change_in_x_local * std::cos(prev_imu_rad) - change_in_y_local * std::sin(prev_imu_rad); // rotate to global
    //     // global_delta_y_inches = change_in_x_local * std::sin(prev_imu_rad) + change_in_y_local * std::cos(prev_imu_rad);
    // }

    // // 7. update global pose
    // robot_pose.x += global_delta_x_inches;
    // robot_pose.y += global_delta_y_inches;
    // // robot_pose.theta is already updated from current_imu_rad

    // // 8. update previous values for next iteration
    // prev_left_encoder_val = current_left_deg;
    // prev_right_encoder_val = current_right_deg;
    // prev_imu_rad = current_imu_rad; // or robot_pose.theta

    // // pros::lcd::print(5, "X:%.1f Y:%.1f H:%.1f", robot_pose.x, robot_pose.y, robot_pose.theta * 180.0/M_PI);
}
*/

// to get the current pose (if odom was working)
/*
Pose get_current_robot_pose() {
    // return robot_pose;
    return {0.0, 0.0, 0.0}; // dummy return since it's not implemented
}
*/

/*
 * to make this real:
 * - uncomment the code sections you need
 * - define your motor/sensor objects and make them accessible (e.g., extern pros::Motor left_drive_motor_for_odom;)
 * - accurately measure and set your constants (WHEEL_DIAMETER_INCHES, TRACK_WIDTH_INCHES, etc.)
 * - create a PROS task in initialize() or main() that calls a function containing update_odometry_loop_content() in a loop 
 *   with a delay (e.g., pros::delay(10) for ~100Hz).
 *   Example task:
 *   static void odom_task_runner(void* param) {
 *       libpid::reset_odometry_tracking(); // or some initial pose
 *       while(true) {
 *           // libpid::update_odometry_loop_content();
 *           pros::delay(10); // run at ~100Hz
 *       }
 *   }
 *   // in initialize():
 *   // pros::Task odom_tracker_task(odom_task_runner, nullptr, "Odom Task");
 */

} // namespace libpid

#endif // LIBPID_ODOMETRY_HPP