// This file is the main sketch that implements the fast serial bridge.
// It uses a very high baud rate (250000) and tokenization for quick command parsing.

#include "ArmController.h"
#include "ArmTest.h"

// --- PIN DEFINITIONS ---
// These pins are hard-coded based on the provided Timer configuration
const int BASE_PIN      = 9;  // Timer1 OC1A
const int SHOULDER_PIN  = 10; // Timer1 OC1B
const int ELBOW_PIN     = 11; // Timer2 OC2A
const int WRIST_PIN     = 3;  // Timer2 OC2B
const int ENABLE_PIN    = 2;  // External enable/power pin (e.g., MOSFET switch)
const int ANGLES_OFFSETS[4] = {28, -65, 15, 150}; // Base, Shoulder, Elbow, Wrist
// Instantiate the arm controller
ArmController arm(BASE_PIN, SHOULDER_PIN, ELBOW_PIN, WRIST_PIN, ENABLE_PIN);

// --- SERIAL COMMAND BUFFER ---
// Keep this small to save RAM
const int MAX_COMMAND_LENGTH = 32;
char command_buffer[MAX_COMMAND_LENGTH];
int buffer_index = 0;

// --- FAST UTILITY FUNCTIONS ---

/**
 * @brief Parses the command buffer to extract four space-separated angle integers.
 * The format is assumed to be: "P B S E W" (e.g., "P 90 45 120 180")
 */
void parseAndExecutePosition() {
    int angles[4] = {0, 0, 0, 0};
    int count = 0;

    // Use strtok to tokenize the string by spaces
    // strtok modifies the input string (command_buffer) in-place.
    // Start after the first character (the 'P' prefix)
    char* token = strtok(command_buffer + 1, " "); 

    // Loop through the four expected tokens
    while (token != NULL && count < 4) {
        // Simple conversion from ASCII string to integer (atoi is fast)
        angles[count++] = atoi(token); 
        token = strtok(NULL, " "); // Get the next token
        
    }

    if (count == 4) {
        // Execute the position command directly to the registers
        // This method has no serial logging for maximum speed.
        arm.moveToPositionSmooth(angles[0] + ANGLES_OFFSETS[0], 
                                angles[1] + ANGLES_OFFSETS[1], 
                                angles[2]  + ANGLES_OFFSETS[2], 
                                angles[3]  + ANGLES_OFFSETS[3],
                                800);

        Serial.print(angles[0]);
        Serial.print(", ");
        Serial.print(angles[1]);
        Serial.print(", ");
        Serial.print(angles[2]);
        Serial.print(", ");
        Serial.println(angles[3]);



        // Serial.println("ACK"); // Uncomment for host acknowledgement
    } else {
        Serial.println("ERR: Invalid P command format. Expected \'P<B> <S> <E> <W>\'");
    }
}

void serialControlBridge() {
     // Non-blocking check for incoming serial data
    while (Serial.available()) {
        char inChar = (char)Serial.read();

        // 1. Check for command termination (Newline character '\n' or Carriage Return '\r')
        if (inChar == '\n' || inChar == '\r') {
            // Null-terminate the string for processing
            command_buffer[buffer_index] = '\0';
            
            // Execute command if the buffer is not empty
            if (buffer_index > 0) {
                char command = command_buffer[0];

                switch (command) {
                    case 'P': // Position command: P<B> <S> <E> <W>
                        parseAndExecutePosition();
                        break;
                        
                    case 'E': // Enable command
                        arm.enableServos();
                        break;
                        
                    case 'D': // Disable command
                        arm.disableServos();
                        break;
                        
                    case 'H': // Home command
                        arm.homeArm();
                        break;
                        
                    default:
                        Serial.print("ERR: Unknown command '");
                        Serial.print(command);
                        Serial.println("\'P<B> <S> <E> <W>\' or \'E\' or \'D\' or \'H\'");
                        break;
                }
            }
            
            // Reset buffer for the next command
            buffer_index = 0;
            
        } 
        
        // 2. Read character and check for buffer overflow
        else if (buffer_index < MAX_COMMAND_LENGTH - 1) {
            command_buffer[buffer_index++] = inChar;
        } 
        
        // 3. Handle overflow (discard the rest of the line until newline)
        else {
            // Drop character. The next newline will reset the buffer.
        }
    }
    
    // The main loop is kept minimal for responsiveness.
}

// --- ARDUINO SETUP AND LOOP ---

void setup() {
    // Use a very high baud rate (250000) for maximum throughput and speed.
    Serial.begin(250000); 
    // Wait for serial port to connect. (Needed for Leonardo/Micro)
    while (!Serial) {;} 
    
    // Initialize the ArmController (sets up timers and home position)
    arm.initialize();
    
    Serial.println("4DOF Serial Bridge Ready. Baud: 250000. Use P<B> <S> <E> <W> or E/D/H commands.");
}

void loop() {
    serialControlBridge();
    // runInteractiveTest();
}


