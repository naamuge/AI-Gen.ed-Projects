#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>

// Q15.16 definitions
typedef int32_t fixed_t;
#define FIXED_BITS 16
#define FIXED_ONE (1 << FIXED_BITS)
#define FLOAT_TO_FIXED(x) ((fixed_t)((x) * FIXED_ONE))
#define FIXED_TO_FLOAT(x) ((float)(x) / FIXED_ONE)
#define FIXED_MULT(x, y) (((int64_t)(x) * (y)) >> FIXED_BITS)

// PID structure
typedef struct {
    // Gains
    fixed_t kp;
    fixed_t ki;
    fixed_t kd;
    fixed_t ka;  // Anti-windup gain
    
    // Setpoint and limits
    fixed_t setpoint;
    fixed_t output_limit_max;
    fixed_t output_limit_min;
    fixed_t integral_deadband;  // Error must exceed this for integral action
    
    // State variables
    fixed_t integral;
    fixed_t prev_error;
    fixed_t prev_deriv;
    fixed_t lpf_coeff;    // Derivative LPF coefficient
} PIDController;

void PID_Init(PIDController *pid);
fixed_t PID_Update(PIDController *pid, fixed_t measurement);

#endif
