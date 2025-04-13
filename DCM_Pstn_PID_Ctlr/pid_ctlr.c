#include "pid_ctlr.h"

// Helper function to constrain value between limits
static fixed_t constrain(fixed_t value, fixed_t min, fixed_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void PID_Init(PIDController *pid) {
    // Convert float values to Q15.16
    pid->kp = FLOAT_TO_FIXED(1.0f);      // Adjust gains as needed
    pid->ki = FLOAT_TO_FIXED(0.1f);
    pid->kd = FLOAT_TO_FIXED(0.01f);
    pid->ka = FLOAT_TO_FIXED(0.1f);      // Anti-windup gain
    
    // LPF coefficient for 10 rad/sec with 100us sample time
    // coeff = 2π * 10 * 0.0001 = 0.00628
    pid->lpf_coeff = FLOAT_TO_FIXED(0.00628f);
    
    // Set limits
    pid->output_limit_max = FLOAT_TO_FIXED(1000.0f);  // Adjust based on your PWM range
    pid->output_limit_min = FLOAT_TO_FIXED(-1000.0f);
    pid->integral_deadband = FLOAT_TO_FIXED(0.01f);   // 1% error deadband
    
    // Reset states
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_deriv = 0;
}

fixed_t PID_Update(PIDController *pid, fixed_t measurement) {
    // Calculate error
    fixed_t error = pid->setpoint - measurement;
    
    // Proportional term
    fixed_t p_term = FIXED_MULT(pid->kp, error);
    
    // Integral term with deadband and anti-windup
    fixed_t i_term;
    if (FIXED_MULT(error, error) > FIXED_MULT(pid->integral_deadband, pid->integral_deadband)) {
        // Only integrate if error exceeds deadband
        pid->integral += FIXED_MULT(pid->ki, error);
    }
    i_term = pid->integral;
    
    // Derivative term with LPF
    fixed_t derivative = error - pid->prev_error;
    fixed_t filtered_deriv = FIXED_MULT(pid->lpf_coeff, derivative) + 
                            FIXED_MULT(FIXED_ONE - pid->lpf_coeff, pid->prev_deriv);
    fixed_t d_term = FIXED_MULT(pid->kd, filtered_deriv);
    
    // Calculate raw output
    fixed_t output = p_term + i_term + d_term;
    
    // Apply output limits
    fixed_t limited_output = constrain(output, pid->output_limit_min, pid->output_limit_max);
    
    // Anti-windup - adjust integral term based on output saturation
    fixed_t windup_error = limited_output - output;
    pid->integral += FIXED_MULT(pid->ka, windup_error);
    
    // Update states
    pid->prev_error = error;
    pid->prev_deriv = filtered_deriv;
    
    return limited_output;
}
