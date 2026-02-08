/**
 * Bluetooth-Controlled Robot Car 
 * 
 * Serial-controlled robot with variable speed control and electronic braking.
 * Supports 8-directional movement (forward, back, left, right, and diagonals)
 * with adjustable speed levels from 0-q (11 speed settings).
 * 
 * Hardware:
 * - L298N Motor Driver (or similar H-bridge)
 * - 2x DC Motors
 * - Bluetooth module (HC-05/HC-06) or USB Serial
 * - Arduino UNO
 * 
 * Serial Commands:
 * - Movement: F (forward), B (back), L (left), R (right)
 * - Diagonals: G (forward-left), I (forward-right), H (back-left), J (back-right)
 * - Speed: 0-9, q (0 = slowest, q = fastest)
 * - Stop: S (with electronic brake)
 * 
 * @author Shayan Mazahir
 * @date of last edit: February 2026
 */

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Motor Driver Control Pins (L298N)
#define in1 5     // Motor A forward
#define in2 6     // Motor A backward
#define in3 10    // Motor B forward
#define in4 11    // Motor B backward
#define LED 13    // Status LED (built-in on most Arduinos)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

int command;                  // Incoming serial command
int Speed = 204;              // Current speed (0-255, default ~80%)
int Speedsec;                 // Secondary speed for turning radius
int buttonState = 0;          // Current command state
int lastButtonState = 0;      // Previous command state (for brake detection)
int Turnradius = 0;           // Turn radius modifier (0-255, must be < Speed)
int brakeTime = 45;           // Electronic brake duration (ms)
int brkonoff = 1;             // Electronic brake enable (1 = on, 0 = off)

// ============================================================================
// SETUP - RUNS ONCE AT STARTUP
// ============================================================================

void setup() {
  // Configure motor control pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(LED, OUTPUT);       // Status LED
  
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
}

// ============================================================================
// MAIN LOOP - RUNS CONTINUOUSLY
// ============================================================================

void loop() {
  // Check if data available on serial port
  if (Serial.available() > 0) {
    command = Serial.read();  // Read incoming byte
    Stop();                   // Stop motors before processing new command
    
    // -------------------------------------------------------------------------
    // COMMAND PROCESSING
    // -------------------------------------------------------------------------
    switch (command) {
      // Basic 4-directional movement
      case 'F':
        forward();
        break;
      case 'B':
        back();
        break;
      case 'L':
        left();
        break;
      case 'R':
        right();
        break;
      
      // Diagonal movement (8-directional control)
      case 'G':
        forwardleft();
        break;
      case 'I':
        forwardright();
        break;
      case 'H':
        backleft();
        break;
      case 'J':
        backright();
        break;
      
      // Speed control (11 levels: 0-9, q)
      // Speed values calibrated for smooth acceleration curve
      case '0':
        Speed = 100;          // ~39% power
        break;
      case '1':
        Speed = 140;          // ~55% power
        break;
      case '2':
        Speed = 153;          // ~60% power
        break;
      case '3':
        Speed = 165;          // ~65% power
        break;
      case '4':
        Speed = 178;          // ~70% power
        break;
      case '5':
        Speed = 191;          // ~75% power
        break;
      case '6':
        Speed = 204;          // ~80% power (default)
        break;
      case '7':
        Speed = 216;          // ~85% power
        break;
      case '8':
        Speed = 229;          // ~90% power
        break;
      case '9':
        Speed = 242;          // ~95% power
        break;
      case 'q':
        Speed = 255;          // 100% power (maximum)
        break;
    }
    
    // Update secondary speed for turn radius calculation
    Speedsec = Turnradius;
    
    // Apply braking system if enabled
    if (brkonoff == 1) {
      brakeOn();
    } else {
      brakeOff();
    }
  }
}

// ============================================================================
// MOVEMENT FUNCTIONS
// ============================================================================

/**
 * Move forward
 * Both motors run forward at current speed
 */
void forward() {
  analogWrite(in1, Speed);    // Motor A forward
  analogWrite(in3, Speed);    // Motor B forward
}

/**
 * Move backward
 * Both motors run in reverse at current speed
 */
void back() {
  analogWrite(in2, Speed);    // Motor A backward
  analogWrite(in4, Speed);    // Motor B backward
}

/**
 * Turn left (pivot)
 * Left motor backward, right motor forward
 */
void left() {
  analogWrite(in3, Speed);    // Motor B forward
  analogWrite(in2, Speed);    // Motor A backward
}

/**
 * Turn right (pivot)
 * Right motor backward, left motor forward
 */
void right() {
  analogWrite(in4, Speed);    // Motor B backward
  analogWrite(in1, Speed);    // Motor A forward
}

/**
 * Move forward-left (diagonal)
 * Left motor at reduced speed for gradual turn
 */
void forwardleft() {
  analogWrite(in1, Speedsec); // Motor A forward (reduced speed)
  analogWrite(in3, Speed);    // Motor B forward (full speed)
}

/**
 * Move forward-right (diagonal)
 * Right motor at reduced speed for gradual turn
 */
void forwardright() {
  analogWrite(in1, Speed);    // Motor A forward (full speed)
  analogWrite(in3, Speedsec); // Motor B forward (reduced speed)
}

/**
 * Move back-right (diagonal)
 * Right motor at reduced speed for gradual turn
 */
void backright() {
  analogWrite(in2, Speed);    // Motor A backward (full speed)
  analogWrite(in4, Speedsec); // Motor B backward (reduced speed)
}

/**
 * Move back-left (diagonal)
 * Left motor at reduced speed for gradual turn
 */
void backleft() {
  analogWrite(in2, Speedsec); // Motor A backward (reduced speed)
  analogWrite(in4, Speed);    // Motor B backward (full speed)
}

/**
 * Stop all motors
 * Sets all motor pins to 0 (coast stop)
 */
void Stop() {
  analogWrite(in1, 0);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
  analogWrite(in4, 0);
}

// ============================================================================
// BRAKING SYSTEM
// ============================================================================

/**
 * Electronic braking system
 * 
 * When 'S' command received, briefly energizes both motor directions
 * simultaneously to create electromagnetic braking effect, then stops.
 * This provides faster deceleration than coast stopping.
 */
void brakeOn() {
  // Read current command state
  buttonState = command;
  
  // Check if state has changed (detect new command)
  if (buttonState != lastButtonState) {
    // If stop command ('S') received
    if (buttonState == 'S') {
      // Verify this is a new stop command (not held)
      if (lastButtonState != buttonState) {
        // Apply electromagnetic brake by energizing all pins
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, HIGH);
        delay(brakeTime);         // Hold brake for specified duration
        Stop();                   // Release brake and coast
      }
    }
    // Update state for next iteration
    lastButtonState = buttonState;
  }
}

/**
 * Brake disabled mode
 * Placeholder function when braking is turned off
 */
void brakeOff() {
  // No action - coast stop only
}
