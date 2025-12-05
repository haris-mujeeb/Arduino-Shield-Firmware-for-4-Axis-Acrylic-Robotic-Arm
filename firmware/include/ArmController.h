#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include <Arduino.h>

// Define the pin mapping based on the provided Timer configuration:
// D9 (Base) -> Timer1 OC1A
// D10 (Shoulder) -> Timer1 OC1B
// D11 (Elbow) -> Timer2 OC2A
// D3 (Wrist) -> Timer2 OC2B
// D2 (Enable) -> Custom pin for MOSFET/H-bridge control

class ArmController {
public:
    // Constructor initializes pin numbers
    ArmController(int baseP, int shoulderP, int elbowP, int wristP, int enableP);

    // Main setup function (configures timers and sets home position)
    void initialize();
    
    // ---------------------------------------------------------
    // CONTROL FUNCTIONS
    // ---------------------------------------------------------
    
    void enableServos();
    void disableServos();
    bool isEnabled() const { return isPowered; } 
    
    /**
     * @brief Instantly sets servos to the target angles.
     * WARNING: This moves at maximum speed. Use for small corrections or initialization.
     */
    void goToPosition(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle);

    /**
     * @brief Moves the arm to the target position using S-Curve Acceleration.
     * This creates a smooth start and stop, reducing mechanical stress.
     * * @param targetBase Target angle for Base
     * @param targetShoulder Target angle for Shoulder
     * @param targetElbow Target angle for Elbow
     * @param targetWrist Target angle for Wrist
     * @param durationMillis Total time the move should take (e.g., 1000 = 1 second)
     */
    void moveToPositionSmooth(int targetBase, int targetShoulder, int targetElbow, int targetWrist, int durationMillis);

    // Moves to home position. stepDelay is used for settling time.
    void homeArm(int stepDelay = 5); 

    // ---------------------------------------------------------
    // READBACK
    // ---------------------------------------------------------
    
    // Reads the current register value and converts back to degrees
    int getCurrentAngle(int index); // 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist
    
private:
    const int basePin;
    const int shoulderPin;
    const int elbowPin;
    const int wristPin;
    const int enablePin;
    
    bool isPowered;

    // Timer configuration and servo control
    void initServosTimer();
    
    // Helper to convert angle to hardware register value
    int angleToTimerValue(int angle, int timer_id);
};

// Global Constants for Home Position
namespace ArmConstants {
    extern const int HOME_BASE; 
    extern const int HOME_SHOULDER; 
    extern const int HOME_ELBOW; 
    extern const int HOME_WRIST; 
}

#endif // ARMCONTROLLER_H