#include "stm32l0xx_hal.h"
#include "pid_ctlr.h"

TIM_HandleTypeDef htim2;  // Encoder timer
TIM_HandleTypeDef htim3;  // PWM timer
TIM_HandleTypeDef htim21; // Sample timer

PIDController pid;
volatile uint8_t pid_flag = 0;

void SystemClock_Config(void);
void GPIO_Init(void);
void Timer_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    Timer_Init();
    
    PID_Init(&pid);
    pid.setpoint = FLOAT_TO_FIXED(1.0f); // 1 revolution setpoint
    
    while (1) {
        if (pid_flag) {
            pid_flag = 0;
            
            // Read encoder (converts to Q15.16)
            int32_t count = TIM2->CNT;
            fixed_t position = FLOAT_TO_FIXED(count / 1024.0f);
            
            // Update PID
            fixed_t output = PID_Update(&pid, position);
            
            // Apply output to PWM (convert back to integer)
            int32_t pwm = output >> FIXED_BITS;
            if (pwm > 0) {
                TIM3->CCR1 = pwm;
                TIM3->CCR2 = 0;
            } else {
                TIM3->CCR1 = 0;
                TIM3->CCR2 = -pwm;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM21) {
        pid_flag = 1;
    }
}

// Timer configurations would go here...
