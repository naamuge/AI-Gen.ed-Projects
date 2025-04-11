#ifndef SERVO_COMMAND_H
#define SERVO_COMMAND_H

#include "servo_control.h"

/* Command Types
 * Enumerates the types of commands that can be sent to the servo:
 * - CMD_SET_ON: Turn the servo on
 * - CMD_SET_OFF: Turn the servo off
 * - CMD_SET_POS: Set the servo position
 * - CMD_SET_SPEED: Set the servo movement speed
 */
typedef enum {
    CMD_SET_ON,    // Turn servo on
    CMD_SET_OFF,   // Turn servo off
    CMD_SET_POS,   // Set servo position
    CMD_SET_SPEED  // Set servo speed
} CommandType;

/* Function Declarations
 * Core functions for processing user commands and monitoring servo state
 */

// Process a single UART command and update the servo state accordingly
void process_uart_command(ServoState* servo);

// Send the current servo state and buffer status over UART
void send_monitoring_data(const ServoState* servo);

#endif // SERVO_COMMAND_H
