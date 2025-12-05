#include "ArmController.h"
#include <Arduino.h> 
#include <avr/io.h>
#include <math.h> // Required for cos() function

// Define the HOME Position constants
namespace ArmConstants {
    const int HOME_BASE = 25;   
    const int HOME_SHOULDER = 0; 
    const int HOME_ELBOW = 60;  
    const int HOME_WRIST = 180; 
}

// ------------------------------------------------------------------
// UTILITY: Timer Mapping
// ------------------------------------------------------------------

int ArmController::angleToTimerValue(int angle, int timer_id) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Timer 1 (16-bit) mapping 
    if (timer_id == 1) {
        return map(angle, 0, 180, 125, 625);
    } 
    
    // Timer 2 (8-bit) mapping
    if (timer_id == 2) {
        return map(angle, 0, 180, 8, 39); 
    }
    return 0;
}

// ------------------------------------------------------------------
// INITIALIZATION
// ------------------------------------------------------------------

void ArmController::initServosTimer() {
    // --- TIMER 1 (16-bit) ---
    TCCR1A = 0; TCCR1B = 0; 
    ICR1 = 4999; 
    TCCR1B |= (1 << WGM13) | (1 << WGM12); 
    TCCR1A |= (1 << WGM11);
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1); 
    TCCR1B |= (1 << CS11) | (1 << CS10); 
    
    // --- TIMER 2 (8-bit) ---
    TCCR2A = 0; TCCR2B = 0;
    TCCR2A |= (1 << WGM21) | (1 << WGM20); 
    TCCR2A |= (1 << COM2A1) | (1 << COM2B1); 
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); 
    
    pinMode(basePin, OUTPUT);
    pinMode(shoulderPin, OUTPUT);
    pinMode(elbowPin, OUTPUT);
    pinMode(wristPin, OUTPUT);
}

ArmController::ArmController(int baseP, int shoulderP, int elbowP, int wristP, int enableP)
    : basePin(baseP), shoulderPin(shoulderP), elbowPin(elbowP), wristPin(wristP), enablePin(enableP),
      isPowered(false) {}

void ArmController::initialize() {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW); 
    initServosTimer();
    goToPosition(ArmConstants::HOME_BASE, ArmConstants::HOME_SHOULDER, ArmConstants::HOME_ELBOW, ArmConstants::HOME_WRIST);
}

// ------------------------------------------------------------------
// MOVEMENT API
// ------------------------------------------------------------------

void ArmController::enableServos() {
    if (!isPowered) {
        digitalWrite(enablePin, HIGH); 
        isPowered = true;
    }
}

void ArmController::disableServos() {
    if (isPowered) {
        digitalWrite(enablePin, LOW); 
        isPowered = false;
    }
}

// Instant movement (Direct Register Write)
void ArmController::goToPosition(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle) {
    OCR1A = angleToTimerValue(baseAngle, 1);     // Base
    OCR1B = angleToTimerValue(shoulderAngle, 1); // Shoulder
    OCR2A = angleToTimerValue(elbowAngle, 2);    // Elbow
    OCR2B = angleToTimerValue(wristAngle, 2);    // Wrist
}

/**
 * @brief Moves the arm from current pos to target pos with S-Curve Acceleration.
 * * @param base/shoulder/elbow/wrist: Target angles
 * @param durationMillis: Total time for the move in milliseconds (e.g., 1000 for 1 second)
 */
void ArmController::moveToPositionSmooth(int targetBase, int targetShoulder, int targetElbow, int targetWrist, int durationMillis) {
    
    // 1. Get current start positions from registers
    int startBase = getCurrentAngle(0);
    int startShoulder = getCurrentAngle(1);
    int startElbow = getCurrentAngle(2);
    int startWrist = getCurrentAngle(3);

    // 2. Initialize timer
    unsigned long startTime = millis();
    
    // 3. Interpolation Loop
    while (true) {
        unsigned long now = millis();
        unsigned long elapsed = now - startTime;
        
        // Check if move is complete
        if (elapsed >= durationMillis) {
            break; 
        }

        // Calculate normalized time 't' (0.0 to 1.0)
        float t = (float)elapsed / (float)durationMillis;

        // --- S-CURVE ACCELERATION MATH ---
        // This formula creates the "Slow Start -> Fast Middle -> Slow End" effect
        float k = (1.0 - cos(t * PI)) / 2.0; 

        // Calculate instantaneous angles based on 'k'
        int nextBase = startBase + (int)((targetBase - startBase) * k);
        int nextShoulder = startShoulder + (int)((targetShoulder - startShoulder) * k);
        int nextElbow = startElbow + (int)((targetElbow - startElbow) * k);
        int nextWrist = startWrist + (int)((targetWrist - startWrist) * k);

        // Update servos using the fast register access
        goToPosition(nextBase, nextShoulder, nextElbow, nextWrist);
        
        // Small delay to allow servo loop to update (approx 60Hz update rate)
        delay(15); 
    }

    // 4. Ensure exact final position is reached (fix rounding errors)
    goToPosition(targetBase, targetShoulder, targetElbow, targetWrist);
}

void ArmController::homeArm(int stepDelay) {
    if (isPowered) return; 
    
    // Use the new smooth move to home if possible, otherwise instant set + power
    delay(5); 
    enableServos(); 
    delay(stepDelay * 20); 
}

int ArmController::getCurrentAngle(int index) {
    if (index == 0) return map(OCR1A, 125, 625, 0, 180); 
    if (index == 1) return map(OCR1B, 125, 625, 0, 180); 
    if (index == 2) return map(OCR2A, 8, 39, 0, 180);    
    if (index == 3) return map(OCR2B, 8, 39, 0, 180);    
    return 90; 
}
