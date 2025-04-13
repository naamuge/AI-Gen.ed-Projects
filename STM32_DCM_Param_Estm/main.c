#include "main.h"
#include "motor_params_rl.h" // Include the RL estimation module header
#include "motor_params_jb.h" // Include the JB estimation module header
#include "debug.h"            // Include the debug print header

// --- Peripheral Handles (Define them here or ensure they are defined in main.h/elsewhere) ---
TIM_HandleTypeDef htim1; // Example: Assuming TIM1 is used for PWM
ADC_HandleTypeDef hadc1; // Example: Assuming ADC1 is used for current

// --- Private function prototypes ---
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

int main(void) {

  // --- MCU Configuration ---
  HAL_Init();
  SystemClock_Config();

  // --- Initialize RTT (conditionally via macro) ---
  DEBUG_INIT();
  DEBUG_PRINTF("SEGGER RTT Initialized (if DEBUG_PRINT is defined).\n");

  // --- Initialize peripherals ---
  MX_GPIO_Init();
  MX_ADC1_Init(); // Initialize ADC used for current sensing
  MX_TIM1_Init(); // Initialize Timer used for PWM

  DEBUG_PRINTF("\n\n--- DC Motor Parameter Estimation ---\n");

  // --- Start necessary peripherals ---
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Start PWM output
  // Calibrate ADC if necessary
  // HAL_ADCEx_Calibration_Start(&hadc1);


  // --- Parameter Estimation Sequence ---

  // ** R and L Estimation (Requires SHAFT LOCKED) **
  DEBUG_PRINTF("\n*** PHASE 1: R/L Estimation (LOCK MOTOR SHAFT!) ***\n");
  HAL_Delay(5000); // Give user time to lock the shaft

  estm_Rs(2.0f); // Estimate R with 2V (adjust voltage as needed)
  HAL_Delay(1000);
  // Choose one inductance estimation method:
  // estm_Ls(5.0f); // Step method
  estm_Ls_sine(3.0f, 100.0f); // Sine method (Example: 3V amplitude, 100 Hz)
  HAL_Delay(1000);

  // ** Ke, B, J Estimation (Requires SHAFT UNLOCKED) **
  DEBUG_PRINTF("\n*** PHASE 2: Ke/B/J Estimation (UNLOCK MOTOR SHAFT!) ***\n");
  HAL_Delay(5000); // Give user time to unlock the shaft

  // Define test parameters (adjust based on your motor and setup)
  float test_vel_for_Ke = 15.0f; // rad/s (approx 143 RPM)
  float test_vel_for_B = 10.0f;  // rad/s (approx 95 RPM) - Use a different speed than Ke
  float test_current_for_J = 0.8f; // Amps - Choose a value that gives reasonable acceleration

  estm_Ke_Kt(test_vel_for_Ke);
  HAL_Delay(1000);

  estm_B(test_vel_for_B);
  HAL_Delay(1000);

  estm_J(test_current_for_J);
  HAL_Delay(1000);

  // --- Print Final Results --- (Using DEBUG_PRINTF macro)
  DEBUG_PRINTF("\n--- Estimation Complete ---\n");
  DEBUG_PRINTF("Estimated R  = %.4f Ohms\n", motor_R);
  DEBUG_PRINTF("Estimated L  = %.6f H\n", motor_L);
  DEBUG_PRINTF("Estimated Ke = %.4f V/(rad/s)\n", motor_Ke);
  DEBUG_PRINTF("Estimated Kt = %.4f Nm/A (Assumed = Ke)\n", motor_Kt);
  DEBUG_PRINTF("Estimated B  = %.6f Nm/(rad/s)\n", motor_B);
  DEBUG_PRINTF("Estimated J  = %.6f kg*m^2\n", motor_J);
  DEBUG_PRINTF("---------------------------\n");

  // --- Main loop ---
  while (1)
  {
    // Stop the motor after estimation for safety
    DC_Ctl_Velo(0.0f); // Use the velocity controller to command zero speed
    HAL_Delay(500);
    // Or directly set PWM to 0 if controllers are just placeholders
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  }
}

// --- System Configuration Functions (Implementations generated by CubeMX or written manually) ---

void SystemClock_Config(void) {
  // ... Clock configuration code ...
}

static void MX_GPIO_Init(void) {
  // ... GPIO initialization code ...
}

static void MX_ADC1_Init(void) {
  // ... ADC1 initialization code (ensure correct channel, trigger, etc.) ...
}

static void MX_TIM1_Init(void) {
  // ... TIM1 initialization code (ensure PWM mode, channel, ARR value) ...
}

void Error_Handler(void) {
  __disable_irq();
  while (1) { }
}
