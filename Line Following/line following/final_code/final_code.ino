/**
 * Line Following Robot - PID Control (Competition Version)
 * 
 * Advanced line follower using PID (Proportional-Integral-Derivative) 
 * control for smooth,accurate line tracking. This version was used
 * in the RobOlympics competition where our team placed 1st.
 * 
 * Algorithm:
 * - Uses proportional control to calculate motor speed adjustments
 * - Smoother curves and faster line following than bang-bang control
 * - Includes line search recovery if robot loses the line
 * 
 * Hardware:
 * - L298N Motor Driver
 * - 2× DC Motors
 * - 2× IR Line Sensors (digital output)
 * - Arduino UNO
 * 
 * Sensor Logic:
 * - 0 = Black (line detected)
 * - 1 = White (no line)
 * 
 * @author Shayan Mazahir
 * @date of last edit: February 2026
 */

#include <L298NX2.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Motor Driver Pins (L298N)
#define ENA 5     // Enable A (left motor speed control - PWM)
#define IN1 8     // Input 1 (left motor direction)
#define IN2 9     // Input 2 (left motor direction)
#define ENB 6     // Enable B (right motor speed control - PWM)
#define IN3 10    // Input 3 (right motor direction)
#define IN4 11    // Input 4 (right motor direction)

// IR Sensor Pins
#define LEFT_IR A0    // Left IR sensor (analog pin used as digital)
#define RIGHT_IR A1   // Right IR sensor (analog pin used as digital)

// ============================================================================
// MOTOR CONTROLLER INITIALIZATION
// ============================================================================

// Create L298NX2 motor controller object
L298NX2 motors(ENA, IN1, IN2, ENB, IN3, IN4);

// ============================================================================
// PID CONTROL PARAMETERS
// ============================================================================

int baseSpeed = 100;    // Base speed for both motors (0-255)
float Kp = 10.0;        // Proportional gain for PID control
                        // Higher = stronger correction, but may oscillate
                        // Lower = smoother but slower correction

// ============================================================================
// SETUP - RUNS ONCE AT STARTUP
// ============================================================================

void setup() {
  // Configure IR sensor pins as inputs
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  
  // Set default motor speed
  motors.setSpeed(baseSpeed);
}

// ============================================================================
// MAIN LOOP - RUNS CONTINUOUSLY
// ============================================================================

void loop() {
  // Read IR sensors
  // 0 = black (line detected), 1 = white (no line)
  int leftValue = digitalRead(LEFT_IR);
  int rightValue = digitalRead(RIGHT_IR);
  
  // -------------------------------------------------------------------------
  // ERROR CALCULATION
  // -------------------------------------------------------------------------
  // Calculate position error relative to line:
  // error = -1 (line is to the left, turn left)
  // error =  0 (centered on line, go straight)
  // error = +1 (line is to the right, turn right)
  int error = leftValue - rightValue;
  
  // -------------------------------------------------------------------------
  // PID CORRECTION
  // -------------------------------------------------------------------------
  // Calculate speed correction based on error
  // Proportional control: correction = Kp × error
  int correction = Kp * error;
  
  // Apply correction to motor speeds
  // Left sensor sees white (+1) → error = +1 → leftSpeed increases, turn left
  // Right sensor sees white (+1) → error = -1 → rightSpeed increases, turn right
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);
  
  // -------------------------------------------------------------------------
  // LINE DETECTION & RECOVERY
  // -------------------------------------------------------------------------
  if (leftValue == 1 && rightValue == 1) {
    // Both sensors on white - line completely lost
    searchForLine();
  } else {
    // At least one sensor on line - apply calculated speeds
    motors.setSpeedA(leftSpeed);   // Left motor
    motors.setSpeedB(rightSpeed);  // Right motor
    motors.forward();              // Move forward with differential speed
  }
}

// ============================================================================
// LINE SEARCH FUNCTION
// ============================================================================

/**
 * Search for line when completely lost
 * 
 * Spins the robot in place until at least one sensor detects
 * the black line again. This recovery behavior prevents the
 * robot from driving off course when it loses the line.
 */
void searchForLine() {
  // Set motors to moderate search speed
  motors.setSpeedA(120);
  motors.setSpeedB(120);
  
  // Spin in place (left motor forward, right motor backward)
  motors.runA(L298N::FORWARD);
  motors.runB(L298N::BACKWARD);
  
  // Keep spinning until at least one sensor finds the black line
  while (digitalRead(LEFT_IR) == 1 && digitalRead(RIGHT_IR) == 1) {
    // Spinning... waiting for line detection
  }
  
  // Stop spinning once line is found
  motors.stop();
  
  // Next loop iteration will resume normal line following
}

