/**
 * Line Following Robot - Testing/Calibration Version
 * 
 * Simple bang-bang control line follower used for initial testing
 * and hardware verification before deploying the PID competition code.
 * 
 * This version uses basic if/else logic to verify:
 * - Motor functionality
 * - Sensor readings
 * - Wiring correctness
 * - Basic line following capability
 * 
 * Algorithm:
 * - Both sensors on black → Go straight
 * - Left sensor on white → Turn left
 * - Right sensor on white → Turn right
 * - Both sensors on white → Stop
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
 * @purpose Testing and hardware verification
 */

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Motor Driver Pins (L298N)
#define enA 5     // Enable A (left motor speed control - PWM)
#define in1 8     // Input 1 (left motor direction)
#define in2 9     // Input 2 (left motor direction)
#define in3 10    // Input 3 (right motor direction)
#define in4 11    // Input 4 (right motor direction)
#define enB 6     // Enable B (right motor speed control - PWM)

// IR Sensor Pins
#define R_S A1    // Right IR sensor
#define L_S A0    // Left IR sensor

// ============================================================================
// SETUP - RUNS ONCE AT STARTUP
// ============================================================================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Configure sensor pins as inputs
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  
  // Configure motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  
  // Enable both motors at full power
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
  
  // Startup delay for safety
  delay(1000);
}

// ============================================================================
// MAIN LOOP - RUNS CONTINUOUSLY
// ============================================================================

void loop() {
  // Read IR sensors
  // 0 = black (line detected), 1 = white (no line)
  int leftSensor = digitalRead(L_S);
  int rightSensor = digitalRead(R_S);
  
  // -------------------------------------------------------------------------
  // DEBUG OUTPUT
  // -------------------------------------------------------------------------
  // Print sensor values to Serial Monitor for debugging
  Serial.print("Left: ");
  Serial.print(leftSensor);
  Serial.print(" Right: ");
  Serial.println(rightSensor);
  
  // -------------------------------------------------------------------------
  // CONTROL LOGIC (Bang-Bang Control)
  // -------------------------------------------------------------------------
  
  // Both sensors on black line → Go straight
  if (leftSensor == 0 && rightSensor == 0) {
    forward();
  }
  // Left sensor on white, right on black → Line is to the right, turn right
  else if (leftSensor == 1 && rightSensor == 0) {
    turnRight();
  }
  // Left sensor on black, right on white → Line is to the left, turn left
  else if (leftSensor == 0 && rightSensor == 1) {
    turnLeft();
  }
  // Both sensors on white → Line lost, stop
  else {
    Stop();
  }
}

// ============================================================================
// MOVEMENT FUNCTIONS
// ============================================================================

/**
 * Move forward
 * Both motors at equal speed (65/255 ≈ 25% power)
 */
void forward() {
  analogWrite(enA, 65);     // Left motor speed
  analogWrite(enB, 65);     // Right motor speed
  digitalWrite(in1, LOW);   // Left motor direction
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);  // Right motor direction
  digitalWrite(in4, LOW);
}

/**
 * Turn right
 * Left motor forward, right motor backward at reduced speed
 */
void turnRight() {
  analogWrite(enA, 45);     // Left motor speed (reduced for turning)
  analogWrite(enB, 45);     // Right motor speed (reduced for turning)
  digitalWrite(in1, LOW);   // Left motor forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);   // Right motor backward (reverse for pivot)
  digitalWrite(in4, HIGH);
}

/**
 * Turn left
 * Right motor forward, left motor backward
 */
void turnLeft() {
  analogWrite(enA, 60);     // Left motor speed
  analogWrite(enB, 60);     // Right motor speed
  digitalWrite(in1, HIGH);  // Left motor backward (reverse for pivot)
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);  // Right motor forward
  digitalWrite(in4, LOW);
}

/**
 * Stop all motors
 * Sets all motor pins to LOW (coast stop)
 */
void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

