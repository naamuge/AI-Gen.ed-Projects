#ifndef INC_MOTOR_PARAMS_JB_H_
#define INC_MOTOR_PARAMS_JB_H_

#include "main.h"
#include "motor_params_rl.h" // Need access to motor_R

// --- Estimated Parameters (Defined in .c file) ---
extern float motor_Ke; // Back-EMF Constant (V/(rad/s))
extern float motor_Kt; // Torque Constant (Nm/A)
extern float motor_J;  // Inertia (kg*m^2)
extern float motor_B;  // Viscous Friction (Nm/(rad/s))

// --- Placeholder Function Prototypes (Implement these based on your hardware/control) ---

/**
 * @brief Reads motor velocity from sensor.
 * @return Velocity in radians per second.
 */
float read_motor_velocity_rad_s(void);

/**
 * @brief Reads the actual voltage applied across the motor terminals.
 * @return Voltage in Volts.
 */
float read_motor_voltage(void);

/**
 * @brief Controls the motor to achieve a target current. (Placeholder)
 * @param target_current Target current in Amps.
 */
void DCM_Ctl_Curr(float target_current);

/**
 * @brief Controls the motor to achieve a target velocity. (Placeholder)
 * @param target_velocity_rad_s Target velocity in rad/s.
 */
void DC_Ctl_Velo(float target_velocity_rad_s);


// --- Public Function Prototypes ---

/**
 * @brief Estimates Back-EMF constant Ke (and assumes Kt = Ke).
 * @note Requires motor shaft to be UNLOCKED and motor_R estimated.
 * @param test_velocity_rad_s Target velocity for the test.
 */
void estimate_Ke_Kt(float test_velocity_rad_s);

/**
 * @brief Estimates viscous friction coefficient B.
 * @note Requires motor shaft to be UNLOCKED and motor_Kt estimated.
 * @param test_velocity_rad_s Target velocity for the test.
 */
void estimate_B(float test_velocity_rad_s);

/**
 * @brief Estimates motor inertia J using a current step.
 * @note Requires motor shaft to be UNLOCKED and motor_Kt, motor_B estimated.
 * @param test_current_step Current step to apply in Amps.
 */
void estimate_J(float test_current_step);


#endif /* INC_MOTOR_PARAMS_JB_H_ */