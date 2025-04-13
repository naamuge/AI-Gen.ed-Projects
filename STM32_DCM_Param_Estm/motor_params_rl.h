#ifndef INC_MOTOR_PARAMS_RL_H_
#define INC_MOTOR_PARAMS_RL_H_

#include "main.h" // Include main HAL types

// --- External Peripheral Handles (Required by the module) ---
extern TIM_HandleTypeDef htim1; // Timer for PWM
extern ADC_HandleTypeDef hadc1; // ADC for measurements

// --- Constants used by RL estimation ---
#define V_SUPPLY 12.0f       // Motor supply voltage (Volts)
#define ADC_RESOLUTION 4096 // 12-bit ADC
#define V_REF 3.3f          // ADC reference voltage (Volts)
#define SHUNT_RESISTOR 0.1f // Shunt resistor value for current sensing (Ohms)
// Note: PWM_MAX_DUTY depends on htim1, calculated internally

// --- Estimated Parameters (Defined in .c file) ---
extern float motor_R; // Estimated Resistance (Ohms)
extern float motor_L; // Estimated Inductance (Henrys)

// --- Public Function Prototypes ---

/**
 * @brief Estimates motor resistance (R).
 * @note Requires motor shaft to be LOCKED.
 * @param test_voltage Voltage to apply during the test.
 */
void estimate_resistance(float test_voltage);

/**
 * @brief Estimates motor inductance (L).
 * @note Requires motor shaft to be LOCKED and motor_R to be estimated first.
 * @param test_voltage Voltage step to apply during the test.
 */
void estimate_inductance(float test_voltage);

#endif /* INC_MOTOR_PARAMS_RL_H_ */