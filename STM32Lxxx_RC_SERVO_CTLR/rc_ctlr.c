#include "servo_control.h"

#define PWM_MIN_DUTY 1000
#define PWM_MAX_DUTY 2000
#define SERVO_MIN_POS 0
#define SERVO_MAX_POS 180

/* Initialize servo state
 * Sets all servo parameters to their default values and
 * initializes the position buffer as empty
 */
void servo_init(ServoState* servo) {
    servo->is_on = false; // Servo is initially off
    servo->current_position_raw = 0; // Raw position starts at 0
    servo->target_position_raw = 0; // Target position starts at 0
    servo->current_pwm_duty = 0; // PWM duty cycle starts at 0
    servo->moving_speed = 1; // Default moving speed is 1
    
    // Initialize position buffer as empty
    servo->pos_buffer.head = 0; // Head of buffer starts at 0
    servo->pos_buffer.tail = 0; // Tail of buffer starts at 0
    servo->pos_buffer.count = 0; // Buffer count starts at 0
}

/* Update servo position
 * This function:
 * 1. Checks if servo is powered on
 * 2. Updates target from buffer if current target is reached
 * 3. Moves current position towards target at specified speed
 * 4. Updates PWM duty cycle based on current position
 */
void servo_update(ServoState* servo) {
    if (!servo->is_on) return; // Do nothing if servo is off

    // Check if current target is reached and buffer has more positions
    if (servo->current_position_raw == servo->target_position_raw && 
        servo->pos_buffer.count > 0) {
        // Get next position from buffer
        servo->target_position_raw = servo->pos_buffer.buffer[servo->pos_buffer.tail];
        servo->pos_buffer.tail = (servo->pos_buffer.tail + 1) % POSITION_BUFFER_SIZE;
        servo->pos_buffer.count--;
    }

    // Update current position towards target
    if (servo->current_position_raw < servo->target_position_raw) {
        // Moving clockwise
        servo->current_position_raw += servo->moving_speed;
        if (servo->current_position_raw > servo->target_position_raw) {
            servo->current_position_raw = servo->target_position_raw; // Prevent overshooting
        }
    } else if (servo->current_position_raw > servo->target_position_raw) {
        // Moving counter-clockwise
        servo->current_position_raw -= servo->moving_speed;
        if (servo->current_position_raw < servo->target_position_raw) {
            servo->current_position_raw = servo->target_position_raw; // Prevent overshooting
        }
    }

    // Convert position to PWM duty cycle and update hardware
    servo->current_pwm_duty = PWM_MIN_DUTY + 
        (servo->current_position_raw * (PWM_MAX_DUTY - PWM_MIN_DUTY) / SERVO_MAX_POS);
    set_pwm_duty_cycle(servo->current_pwm_duty); // Update hardware with new duty cycle
}

/* Set servo power state
 * Controls whether the servo is actively maintaining position
 */
void servo_set_state(ServoState* servo, bool state) {
    servo->is_on = state; // Update servo power state
}

/* Add new position to buffer
 * Returns false if buffer is full or position is invalid
 */
bool servo_add_position_to_buffer(ServoState* servo, uint16_t position) {
    if (servo->pos_buffer.count >= POSITION_BUFFER_SIZE || 
        position > SERVO_MAX_POS) {
        return false; // Return false if buffer is full or position is invalid
    }
    
    servo->pos_buffer.buffer[servo->pos_buffer.head] = position; // Add position to buffer
    servo->pos_buffer.head = (servo->pos_buffer.head + 1) % POSITION_BUFFER_SIZE; // Update buffer head
    servo->pos_buffer.count++; // Increment buffer count
    return true; // Return true if position was successfully added
}

/* Set new target position
 * If buffer is empty, sets immediate target
 * Otherwise, adds to position buffer
 */
void servo_set_position(ServoState* servo, uint16_t position) {
    if (position <= SERVO_MAX_POS) { // Check if position is valid
        if (servo->pos_buffer.count == 0) {
            servo->target_position_raw = position; // Set immediate target if buffer is empty
        } else {
            servo_add_position_to_buffer(servo, position); // Add position to buffer
        }
    }
}

/* Set servo movement speed
 * Speed determines how many position units to move per update
 */
void servo_set_speed(ServoState* servo, uint8_t speed) {
    if (speed > 0) {
        servo->moving_speed = speed; // Update moving speed if valid
    }
}
