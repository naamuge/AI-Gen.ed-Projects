#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* PWM Configuration Constants
 * PWM_MIN/MAX_DUTY: Define the pulse width range for servo control (in microseconds)
 * SERVO_MIN/MAX_POS: Define the angular range of servo movement (in degrees)
 */
#define PWM_MIN_DUTY 1000  // 1ms pulse width - typically 0 degrees
#define PWM_MAX_DUTY 2000  // 2ms pulse width - typically 180 degrees
#define SERVO_MIN_POS 0    // Minimum angle in degrees
#define SERVO_MAX_POS 180  // Maximum angle in degrees
#define POSITION_BUFFER_SIZE 10  // Size of position command queue

/* Position Ring Buffer Structure
 * Implements a circular buffer to store upcoming position commands
 * This allows queuing of multiple position commands for sequential execution
 */
typedef struct {
    uint16_t buffer[POSITION_BUFFER_SIZE];  // Array to store position values
    uint8_t head;     // Index for next write position
    uint8_t tail;     // Index for next read position
    uint8_t count;    // Number of positions currently in buffer
} PositionBuffer;

/* Servo State Structure
 * Maintains complete state of servo including:
 * - Current operating status
 * - Position tracking (current and target)
 * - Movement parameters
 * - Position command buffer
 */
typedef struct {
    bool is_on;                    // Servo power state
    uint16_t current_position_raw; // Current angular position (0-180)
    uint16_t target_position_raw;  // Target position to move to
    uint16_t current_pwm_duty;     // Current PWM duty cycle
    uint8_t moving_speed;          // Movement speed (positions per update)
    PositionBuffer pos_buffer;     // Buffer for queued positions
} ServoState;

/* Function Declarations
 * Core functions for servo control and state management
 */

// Initialize servo state and position buffer
void servo_init(ServoState* servo);

// Update servo position based on current state and parameters
void servo_update(ServoState* servo);

// Set servo power state (ON/OFF)
void servo_set_state(ServoState* servo, bool state);

// Set new target position (immediate or buffered)
void servo_set_position(ServoState* servo, uint16_t position);

// Set servo movement speed
void servo_set_speed(ServoState* servo, uint8_t speed);

// Add a new position to the buffer queue
bool servo_add_position_to_buffer(ServoState* servo, uint16_t position);

#endif // SERVO_CONTROL_H
