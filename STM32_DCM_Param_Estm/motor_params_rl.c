#include "motor_params_rl.h"
#include <math.h>
#include "debug.h" // Include the debug print header

// --- Estimated Parameters (Definition) ---
float motor_R = 0.0f;
float motor_L = 0.0f;

// --- Internal Helper Functions ---

// Function to set motor voltage via PWM
static void set_Vs(float voltage) {
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > V_SUPPLY) voltage = V_SUPPLY;
    // Calculate max duty based on the specific timer instance
    uint32_t pwm_max_duty = (htim1.Instance->ARR);
    uint32_t duty = (uint32_t)((voltage / V_SUPPLY) * pwm_max_duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

// Function to read ADC value and convert to current
static float set_Is() {
    uint32_t adc_value = 0;
    // Assuming ADC is configured for single conversion, software trigger
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1); // Stop ADC after conversion

    float voltage_at_shunt = ((float)adc_value / ADC_RESOLUTION) * V_REF;
    // Adjust calculation if using an amplifier for current sense
    float current = voltage_at_shunt / SHUNT_RESISTOR;
    return current;
}

// --- Public Functions ---

void estm_Rs(float test_voltage) {
    DEBUG_PRINTF("Estimating R (Shaft LOCKED!). Applying %.2fV...\n", test_voltage);
    set_Vs(test_voltage);
    HAL_Delay(1000); // Wait for current to stabilize

    float steady_current = 0.0f;
    int samples = 10;
    for (int i = 0; i < samples; i++) {
        steady_current += set_Is();
        HAL_Delay(10); // Small delay between samples
    }
    steady_current /= samples;

    set_Vs(0); // Turn off motor
    DEBUG_PRINTF("Steady current measured: %.3f A\n", steady_current);

    if (steady_current > 0.01f) { // Avoid division by zero/small numbers
        motor_R = test_voltage / steady_current;
        DEBUG_PRINTF("Estimated R: %.4f Ohms\n", motor_R);
    } else {
        motor_R = 0.0f; // Indicate error or invalid measurement
        DEBUG_PRINTF("Resistance estimation failed (current too low).\n");
    }
}

void estm_Ls(float test_voltage) {
    if (motor_R < 0.001f) {
        DEBUG_PRINTF("Inductance estimation skipped: Valid R required first.\n");
        return;
    }
    DEBUG_PRINTF("Estimating L (Shaft LOCKED!). Applying %.2fV step...\n", test_voltage);

    set_Vs(0);
    HAL_Delay(500); // Ensure motor is off

    uint32_t start_time = HAL_GetTick();
    set_Vs(test_voltage);

    float current = 0.0f;
    // Target current is 63.2% of the final steady-state current (V/R)
    float target_current = (test_voltage / motor_R) * (1.0f - expf(-1.0f));
    uint32_t time_at_target = 0;
    uint32_t timeout_ms = 500; // Timeout for the measurement

    DEBUG_PRINTF("Target current for L (63.2%%): %.3f A\n", target_current);

    while (HAL_GetTick() - start_time < timeout_ms) {
        current = set_Is();
        if (current >= target_current) {
            time_at_target = HAL_GetTick();
            break;
        }
        // A small delay might be needed if ADC sampling is very fast
        // HAL_Delay(1);
    }

    set_Vs(0); // Turn off motor

    if (time_at_target > 0) {
        float time_constant_ms = (float)(time_at_target - start_time);
        float time_constant_s = time_constant_ms / 1000.0f;
        motor_L = motor_R * time_constant_s; // Time constant Tau = L/R
        DEBUG_PRINTF("Time to reach target current: %.1f ms\n", time_constant_ms);
        DEBUG_PRINTF("Estimated L: %.6f H\n", motor_L);
    } else {
        motor_L = 0.0f; // Indicate error or timeout
        DEBUG_PRINTF("Inductance estimation failed (timeout or current didn't reach target).\n");
    }
}

/**
 * @brief Estimates motor inductance (Ls) using sinusoidal voltage input.
 * @note Requires motor shaft to be LOCKED and motor_R to be estimated first.
 * @param test_voltage_amplitude Amplitude of the sine voltage to apply (Volts).
 * @param frequency_hz Frequency of the sine wave (Hz).
 */
 void estm_Ls_sine(float test_voltage_amplitude, float frequency_hz) {
    if (motor_R < 0.001f) {
        DEBUG_PRINTF("Inductance estimation skipped: Valid R required first.\n");
        return;
    }
    if (frequency_hz <= 0) {
         DEBUG_PRINTF("Inductance estimation skipped: Frequency must be positive.\n");
         return;
    }
     if (test_voltage_amplitude <= 0 || test_voltage_amplitude > V_SUPPLY) {
         DEBUG_PRINTF("Inductance estimation skipped: Invalid voltage amplitude.\n");
         return;
    }

    DEBUG_PRINTF("Estimating L (Shaft LOCKED!) using Sine Wave...\n");
    DEBUG_PRINTF("Applying %.2fV amplitude at %.1f Hz...\n", test_voltage_amplitude, frequency_hz);

    // Angular frequency
    float omega = 2.0f * M_PI * frequency_hz;

    // Duration of the test - apply for a few cycles
    uint32_t duration_ms = (uint32_t)(5.0f / frequency_hz * 1000.0f); // 5 cycles
    if (duration_ms < 500) duration_ms = 500; // Minimum duration
    if (duration_ms > 5000) duration_ms = 5000; // Maximum duration

    DEBUG_PRINTF("Test duration: %lu ms\n", duration_ms);

    float max_current_amplitude = 0.0f;
    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = 0;

    // --- Apply Sinusoidal Voltage and Measure Current ---
    while ((current_time = HAL_GetTick()) - start_time < duration_ms) {
        // Calculate instantaneous voltage based on sine wave
        float time_s = (float)(current_time - start_time) / 1000.0f;
        float instantaneous_voltage = test_voltage_amplitude * sinf(omega * time_s);

        // Apply the voltage (ensure it's positive for simple PWM)
        // This applies a rectified sine wave if only one PWM channel is used.
        // For a true sine wave across H-bridge, control logic is more complex.
        // We assume the positive half-cycle is sufficient if V_amplitude <= V_SUPPLY.
        set_Vs(instantaneous_voltage > 0 ? instantaneous_voltage : 0);

        // Measure instantaneous current
        float current = set_Is();

        // Track the peak current - simple peak detection
        if (current > max_current_amplitude) {
            max_current_amplitude = current;
        }

        // Small delay to allow PWM cycle and ADC conversion, adjust as needed
        // This delay affects the effective sine wave frequency if too large.
        HAL_Delay(1);
    }

    set_Vs(0); // Turn off motor
    DEBUG_PRINTF("Maximum current amplitude measured: %.3f A\n", max_current_amplitude);

    // --- Calculate Inductance ---
    if (max_current_amplitude > 0.01f) {
        // Calculate impedance Z = V_amplitude / I_amplitude
        float impedance_Z = test_voltage_amplitude / max_current_amplitude;
        DEBUG_PRINTF("Calculated Impedance (Z): %.4f Ohms\n", impedance_Z);

        // Ensure Z is not less than R (physically impossible)
        if (impedance_Z < motor_R) {
             DEBUG_PRINTF("Inductance estimation failed: Calculated Z (%.4f) < R (%.4f).\n", impedance_Z, motor_R);
             motor_L = 0.0f; // Error
        } else {
            // Calculate L from Z = sqrt(R^2 + (omega*L)^2)
            // (omega*L)^2 = Z^2 - R^2
            // L = sqrt(Z^2 - R^2) / omega
            float omegaL_squared = (impedance_Z * impedance_Z) - (motor_R * motor_R);
            if (omegaL_squared < 0) omegaL_squared = 0; // Clamp if Z slightly less than R due to noise

            motor_L = sqrtf(omegaL_squared) / omega;
            DEBUG_PRINTF("Estimated L: %.6f H\n", motor_L);
        }
    } else {
        motor_L = 0.0f; // Indicate error or invalid measurement
        DEBUG_PRINTF("Inductance estimation failed (current amplitude too low).\n");
    }
}

