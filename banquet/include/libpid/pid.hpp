#ifndef LIBPID_PID_HPP
#define LIBPID_PID_HPP

#include <cmath> 

namespace libpid {

class PIDController {
public:
    // your pid constants
    double kP, kI, kD;

    // internal stuff for the pid math
    double integral;
    double prev_error;

    // clamp the output 
    double min_output;
    double max_output;

    // stops the integral term from od high (integral windup)
    double max_integral_contribution; // max value i_out can be

    PIDController(double p, double i, double d, 
                  double min_out = -12000.0, // vex motors take -12000mV to 12000mV
                  double max_out = 12000.0,
                  double max_i_contrib = 4000.0) // e.g., limit integral to 1/3 of max output
        : kP(p), kI(i), kD(d), 
          integral(0.0), prev_error(0.0),
          min_output(min_out), max_output(max_out),
          max_integral_contribution(max_i_contrib) {
        // constructor, pretty straightforward
    }

    // call this when you start a new movement to clear out old values
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }

    // calculates what power to send to motors
    double calculate(double setpoint, double current_value) {
        double error = setpoint - current_value;

        // proportional term, how far off 
        double p_out = kP * error;

        // integral term, if bot been off path for a min increase term
        integral += error;
        double i_out = kI * integral;

        // anti-windup for integral, clamp its contribution
        if (i_out > max_integral_contribution) {
            i_out = max_integral_contribution;
            integral = i_out / kI; // also cap the raw integral to prevent future jumps if kI changes
        } else if (i_out < -max_integral_contribution) {
            i_out = -max_integral_contribution;
            integral = i_out / kI;
        }
        
        // derivative term: how fast is bot closing in (or overshooting)
        double derivative = error - prev_error;
        double d_out = kD * derivative;

        // sum it all up
        double output = p_out + i_out + d_out;

        // remember this error for next time's derivative calc
        prev_error = error;

        // make sure output is within the min/max limits
        if (output > max_output) {
            output = max_output;
        } else if (output < min_output) {
            output = min_output;
        }

        return output;
    }

    // if you wanna tweak constants 
    void set_constants(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        // might wanna reset integral/prev_error if you change constants drastically
    }
};

} // namespace libpid

#endif 