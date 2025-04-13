#include "motor_params_jb.h"
#include <math.h>
#include "debug.h" // Include the debug print header

// --- Estimated Parameters (Definition) ---
float motor_Ke = 0.0f;
float motor_Kt = 0.0f;
float motor_J = 0.0f;
float motor_B = 0.0f;

// --- Placeholder Function Implementations (Replace with actual code) ---

// Reads velocity from an encoder or sensor (radians/second)
float read_velo() {
    // *** Replace with your actual encoder reading and conversion logic ***
    // Example: return (float)read_encoder_ticks() * GEAR_RATIO * (2.0f * M_PI / ENCODER_CPR) / TIME_DELTA_S;
    DEBUG_PRINTF("Warning: Using placeholder read_velo()\n");
    return 0.0f;
}

// Reads the actual voltage applied to the motor terminals
float read_Vs() {
    // *** Replace with your actual voltage measurement or estimation logic ***
    // Example: return V_SUPPLY * (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) / (float)(htim1.Instance->ARR));
    DEBUG_PRINTF("Warning: Using placeholder read_Vs()\n");
     return 0.0f;
}

// Ideal current controller (Placeholder - replace with real controller)
void DCM_Ctl_Curr(float target_current) {
    // *** Replace with your actual current control loop ***
    DEBUG_PRINTF("Placeholder: Setting current to: %.3f A\n", target_current);
    // Example conceptual action: Adjust PWM until read_motor_current() matches target_current
    // For now, just set voltage proportional to current for basic simulation
    // This is NOT a real current controller! Needs motor_R from rl module.
    // float target_voltage = target_current * motor_R; // Highly simplified! Ignores L and back-EMF
    // set_motor_voltage(target_voltage); // Need set_motor_voltage or direct PWM control here
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Default to off
}

// Ideal velocity controller (Placeholder - replace with real controller)
void DC_Ctl_Velo(float target_velocity_rad_s) {
    // *** Replace with your actual velocity control loop ***
    DEBUG_PRINTF("Placeholder: Setting velocity to: %.3f rad/s\n", target_velocity_rad_s);
    // Example conceptual action: Adjust PWM until read_velo() matches target_velocity_rad_s
    // For now, just set voltage proportional to velocity for basic simulation
    // This is NOT a real velocity controller! Needs motor_Ke, motor_R.
    // float target_voltage = target_velocity_rad_s * motor_Ke + motor_R * 0.1f; // Highly simplified! Assumes Ke known, low current
    // set_motor_voltage(target_voltage); // Need set_motor_voltage or direct PWM control here
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Default to off
}

// --- Public Functions ---

void estm_Ke_Kt(float test_velocity_rad_s) {
    if (motor_R < 0.001f) {
         DEBUG_PRINTF("Ke/Kt estimation skipped: Valid R required first.\n");
         return;
    }

    DEBUG_PRINTF("Estimating Ke/Kt (Shaft UNLOCKED!). Commanding %.2f rad/s...\n", test_velocity_rad_s);
    DC_Ctl_Velo(test_velocity_rad_s); // Command velocity
    HAL_Delay(2000); // Wait for steady state (adjust delay as needed)

    float steady_velocity = read_velo(); // Verify actual velocity
    float steady_current = read_motor_current(); // Need current reading from rl module or separate function
    float steady_voltage = read_Vs(); // Need to know voltage applied

    DEBUG_PRINTF("Steady state: Vel=%.3f rad/s, Curr=%.3f A, Volt=%.2f V\n",
           steady_velocity, steady_current, steady_voltage);

    // Optional: Add checks if steady_velocity is close enough to test_velocity_rad_s

    if (fabsf(steady_velocity) > 0.01f) { // Avoid division by zero
        float back_emf = steady_voltage - steady_current * motor_R;
        motor_Ke = fabsf(back_emf / steady_velocity); // Ke is typically positive
        motor_Kt = motor_Ke; // Assume Kt = Ke (SI units)
        DEBUG_PRINTF("Estimated Ke: %.4f V/(rad/s), Assumed Kt: %.4f Nm/A\n", motor_Ke, motor_Kt);
    } else {
        motor_Ke = 0.0f;
        motor_Kt = 0.0f;
        DEBUG_PRINTF("Ke/Kt estimation failed (velocity too low or measurement error).\n");
    }
    DC_Ctl_Velo(0.0f); // Stop motor
    HAL_Delay(500);
}

void estm_Bm(float test_velocity_rad_s) {
    if (motor_Kt < 0.0001f) {
        DEBUG_PRINTF("B estimation skipped: Valid Kt required first.\n");
        return;
    }

    DEBUG_PRINTF("Estimating B (Shaft UNLOCKED!). Commanding %.2f rad/s...\n", test_velocity_rad_s);
    DC_Ctl_Velo(test_velocity_rad_s); // Command velocity
    HAL_Delay(2000); // Wait for steady state

    float steady_velocity = read_velo();
    float steady_current = read_motor_current(); // Need current reading

    DEBUG_PRINTF("Steady state for B: Vel=%.3f rad/s, Curr=%.3f A\n", steady_velocity, steady_current);

    // Optional: Add checks if steady_velocity is close enough to test_velocity_rad_s

    if (fabsf(steady_velocity) > 0.01f) {
        // At steady state, Torque_motor = Torque_friction
        // Torque_motor = Kt * I
        // Torque_friction = B * omega (viscous friction)
        float friction_torque = motor_Kt * steady_current;
        motor_B = fabsf(friction_torque / steady_velocity); // B is typically positive
        DEBUG_PRINTF("Estimated B: %.6f Nm/(rad/s)\n", motor_B);
    } else {
        motor_B = 0.0f;
        DEBUG_PRINTF("B estimation failed (velocity too low or measurement error).\n");
    }
    DC_Ctl_Velo(0.0f); // Stop motor
    HAL_Delay(500);
}

void estm_Jm(float test_current_step) {
     if (motor_Kt < 0.0001f || motor_B < 0.0f) { // B can be zero, but check Kt
        DEBUG_PRINTF("J estimation skipped: Valid Kt and B required first.\n");
        return;
     }

    DEBUG_PRINTF("Estimating J (Shaft UNLOCKED!). Applying %.2f A current step...\n", test_current_step);
    // Ensure motor is stopped
    DC_Ctl_Velo(0.0f);
    HAL_Delay(1000);

    float initial_velocity = read_velo();
    uint32_t t1 = HAL_GetTick();

    DCM_Ctl_Curr(test_current_step); // Apply current step -> torque step

    // Need to measure velocity change shortly after step
    // This requires fast sampling. Delay is illustrative and CRITICAL.
    HAL_Delay(20); // *** ADJUST THIS DELAY based on expected acceleration! ***

    float final_velocity = read_velo();
    uint32_t t2 = HAL_GetTick();

    DCM_Ctl_Curr(0.0f); // Stop applying current
    DC_Ctl_Velo(0.0f); // Command zero velocity for safety

    float delta_t_ms = (float)(t2 - t1);
    if (delta_t_ms < 1.0f) delta_t_ms = 1.0f; // Avoid division by zero if ticks are the same
    float delta_t_s = delta_t_ms / 1000.0f;

    // Calculate acceleration (alpha = delta_omega / delta_t)
    float acceleration = (final_velocity - initial_velocity) / delta_t_s;

    // Calculate average velocity during the short interval to estimate friction torque
    float avg_velocity = (final_velocity + initial_velocity) / 2.0f;

    // Calculate applied torque and friction torque
    float applied_torque = motor_Kt * test_current_step;
    float friction_torque = motor_B * avg_velocity; // Use estimated B

    // Dynamic equation: T_applied - T_friction = J * alpha
    float net_torque = applied_torque - friction_torque;

    DEBUG_PRINTF("J estimation details: dt=%.1fms, v_init=%.3f, v_final=%.3f, acc=%.2f\n",
           delta_t_ms, initial_velocity, final_velocity, acceleration);
    DEBUG_PRINTF("T_app=%.4f, T_fric=%.4f, T_net=%.4f\n",
           applied_torque, friction_torque, net_torque);


    if (fabsf(acceleration) > 0.01f) { // Avoid division by zero or noise amplification
        motor_J = net_torque / acceleration;
        if (motor_J < 0) { // Inertia should not be negative
             DEBUG_PRINTF("J estimation resulted in negative value (%.6f) - likely measurement error.\n", motor_J);
             motor_J = 0.0f;
        } else {
            DEBUG_PRINTF("Estimated J: %.6f kg*m^2\n", motor_J);
        }
    } else {
        motor_J = 0.0f;
        DEBUG_PRINTF("J estimation failed (acceleration too low or measurement error).\n");
    }
    HAL_Delay(500);
}