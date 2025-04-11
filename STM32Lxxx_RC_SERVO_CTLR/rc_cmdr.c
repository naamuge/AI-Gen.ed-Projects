#include "servo_command.h"

/* Process UART Commands
 * Reads and processes commands from UART with the following format:
 * "ON" - Turn servo on
 * "OFF" - Turn servo off
 * "POS xxx" - Set position to xxx degrees
 * "SPD xxx" - Set speed to xxx units
 */
void process_uart_command(ServoState* servo) {
    char command[64]; // Buffer to store incoming command
    int index = 0;    // Current index in the command buffer
    int byte;         // Byte received from UART

    // Read bytes until newline or buffer full
    while ((byte = uart_receive_byte()) != -1) {
        if (byte == '\n') {
            command[index] = '\0'; // Null-terminate the command string
            
            // Process complete command
            if (strcmp(command, "ON") == 0) {
                servo_set_state(servo, true); // Turn servo on
            } 
            else if (strcmp(command, "OFF") == 0) {
                servo_set_state(servo, false); // Turn servo off
            }
            else if (strncmp(command, "POS", 3) == 0) {
                // Extract position value after "POS "
                uint16_t position = atoi(&command[4]);
                servo_set_position(servo, position); // Set servo position
            }
            else if (strncmp(command, "SPD", 3) == 0) {
                // Extract speed value after "SPD "
                uint8_t speed = atoi(&command[4]);
                servo_set_speed(servo, speed); // Set servo speed
            }
            index = 0; // Reset buffer index for next command
        } else {
            if (index < 63) { // Prevent buffer overflow
                command[index++] = (char)byte;
            }
        }
    }
}

/* Send Monitoring Data
 * Outputs current servo state including:
 * - Power state (ON/OFF)
 * - Current position
 * - Target position
 * - Movement speed
 * - Number of buffered positions
 */
void send_monitoring_data(const ServoState* servo) {
    char buffer[128]; // Buffer to store monitoring message
    snprintf(buffer, sizeof(buffer), 
             "Servo State: ON=%d, Pos=%d, Target=%d, Speed=%d, Buffered=%d\n",
             servo->is_on, 
             servo->current_position_raw, 
             servo->target_position_raw, 
             servo->moving_speed, 
             servo->pos_buffer.count);
    uart_send_string(buffer); // Send monitoring data over UART
}
