#include <Servo.h>
#include <Wire.h>
#include "MT6701.h"
#include "ArduPID.h"

// ========== ADJUSTABLE PARAMETERS (EASY ACCESS) ==========
const float targetDist = 888.0;           // Target distance in centimeters
const float targetTime = 15.0;            // Target time in seconds to reach distance
const int steeringAngle = 90;             // Steering servo angle (90 = straight)
const float decelerationDist = 5.0;       // Distance over which deceleration occurs in centimeters
// =========================================================

// Servo and Encoder
Servo esc;  // create servo object to control the ESC
Servo steeringServo;  // steering servo on pin 9
MT6701 encoder;

// PID Controllers
ArduPID positionPID;  // PID for final position control
ArduPID velocityPID;  // PID for velocity control during cruise

// Pin definitions
int pwmPin = 10;            // ESC input connected to digital pin 10
int steeringServoPin = 9;   // Steering servo on digital pin 9
const int button1 = 7;      // Button to toggle laser
const int button2 = 4;      // Button to start/stop driving
const int laser = 8;        // Laser control pin

// Pulse width limits for ESC
int minPulseWidth = 1500;   // Minimum pulse width (corresponding to minimum speed)
int maxPulseWidth = 2000;   // Maximum pulse width (corresponding to maximum speed)

// Wheel and gear constants
const float wheelDiameterInches = 2.875;                                      // Wheel diameter in inches
const float gearRatio = 1.2;                                                  // Gear ratio (encoder to wheel)
const float wheelCircumference = wheelDiameterInches * 2.54 * 3.14159265359;  // Circumference of the wheel in centimeters

// PID variables
double posSetpoint, posInput, posOutput;
double velSetpoint, velInput, velOutput;

// Calculated target velocity
float targetVelocity;  // cm/s

// State variables
float previousAngle = 0.0;
float cumulativeDistance = 0.0;
float currentVelocity = 0.0;
float power = 0.0;
bool driving = false;  // State variable to control driving
bool positionPIDInitialized = false;  // Track whether position PID has been initialized
bool velocityPIDInitialized = false;  // Track whether velocity PID has been initialized
bool decelerating = false;
bool laserOn = false;  // Laser state
bool encoderInitialized = false;

// Timing variables
unsigned long lastLoopTime = 0;
float deltaTime = 0.0;

// Button debouncing variables
int button1State;
int lastButton1State = HIGH;  // Initial state (HIGH because of INPUT_PULLUP)
int button2State;
int lastButton2State = HIGH;  // Initial state (HIGH because of INPUT_PULLUP)
unsigned long lastDebounceTime1 = 0;  // The last time the output pin was toggled
unsigned long debounceDelay1 = 30;    // The debounce time; increase if the output flickers
unsigned long lastDebounceTime2 = 0;  // The last time the output pin was toggled
unsigned long debounceDelay2 = 30;    // The debounce time; increase if the output flickers

void setup() {
  Serial.begin(500000);
  Wire.begin();
  encoder.initializeI2C();
  previousAngle = encoder.angleRead();
  
  // Attach and initialize servos
  steeringServo.attach(steeringServoPin);
  steeringServo.write(steeringAngle);  // Set to configured steering angle
  
  esc.attach(pwmPin);
  esc.writeMicroseconds(1500);  // Initialize ESC with 1ms pulse width
  
  pinMode(button1, INPUT_PULLUP);  // Use the internal pull-up resistor
  pinMode(button2, INPUT_PULLUP);  // Use the internal pull-up resistor
  pinMode(laser, OUTPUT);

  // Calculate target velocity
  targetVelocity = targetDist / targetTime;  // cm/s

  // Initialize Position PID (for final deceleration)
  double posKp = 0.002, posKi = 0.0, posKd = 0.0;
  positionPID.begin(&posInput, &posOutput, &posSetpoint, posKp, posKi, posKd);
  positionPID.setOutputLimits(-1.0, 1.0);

  // Initialize Velocity PID (for cruise control)
  double velKp = 0.01, velKi = 0.001, velKd = 0.0;
  velocityPID.begin(&velInput, &velOutput, &velSetpoint, velKp, velKi, velKd);
  velocityPID.setOutputLimits(0.0, 1.0);
  
  // Print setup information
  Serial.print("Target distance: ");
  Serial.print(targetDist);
  Serial.println(" cm");
  Serial.print("Target time: ");
  Serial.print(targetTime);
  Serial.println(" s");
  Serial.print("Target velocity: ");
  Serial.print(targetVelocity);
  Serial.println(" cm/s");
  
  lastLoopTime = millis();
}

void loop() {
  // Calculate delta time
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastLoopTime) / 1000.0;  // Convert to seconds
  lastLoopTime = currentTime;

  // Debounce logic for button1 (laser toggle)
  int reading1 = digitalRead(button1);
  if (reading1 != lastButton1State) {
    lastDebounceTime1 = millis();
  }
  if ((millis() - lastDebounceTime1) > debounceDelay1) {
    if (reading1 != button1State) {
      button1State = reading1;
      if (button1State == LOW) {
        laserOn = !laserOn;
      }
    }
  }
  digitalWrite(laser, laserOn ? HIGH : LOW);
  lastButton1State = reading1;

  // Debounce logic for button2 (driving toggle)
  int reading2 = digitalRead(button2);
  if (reading2 != lastButton2State) {
    lastDebounceTime2 = millis();
  }
  if ((millis() - lastDebounceTime2) > debounceDelay2) {
    if (reading2 != button2State) {
      button2State = reading2;
      if (button2State == LOW) {
        driving = !driving;
        // Reset state when stopping
        if (!driving) {
          velocityPIDInitialized = false;
          positionPIDInitialized = false;
          decelerating = false;
          encoderInitialized = false;
          velocityPID.stop();
          positionPID.stop();
          esc.writeMicroseconds(1500);  // Stop motor
        }
      }
    }
  }
  lastButton2State = reading2;

  // Driving logic
  if (driving) {
    float currentAngle = encoder.angleRead();
    float angleDifference = currentAngle - previousAngle;

    // Handle the wrap-around from 360 to 0 degrees
    if (angleDifference < -180.0) {
      angleDifference += 360.0;
    } else if (angleDifference > 180.0) {
      angleDifference -= 360.0;
    }

    // Calculate distance traveled in this loop iteration
    float distanceTraveled = (angleDifference / 360.0) * (wheelCircumference / gearRatio);
    cumulativeDistance -= distanceTraveled;

    if (!encoderInitialized){
      cumulativeDistance = 0;
      encoderInitialized = true;
    }

    // Calculate current velocity (cm/s)
    if (deltaTime > 0) {
      currentVelocity = -distanceTraveled / deltaTime;
    }

    // Determine which control mode to use
    if (cumulativeDistance < targetDist - decelerationDist) {
      // Velocity control mode - maintain target velocity
      if (!velocityPIDInitialized) {
        velSetpoint = targetVelocity;
        velocityPID.start();
        velocityPIDInitialized = true;
      }
      
      velInput = currentVelocity;
      velocityPID.compute();
      power = velOutput;
      
    } else {
      // Position control mode - decelerate to target position
      if (!positionPIDInitialized) {
        posSetpoint = targetDist;
        positionPID.start();
        positionPIDInitialized = true;
        decelerating = true;
        velocityPID.stop();
      }
      
      posInput = cumulativeDistance;
      positionPID.compute();
      power = posOutput;
    }

    // Add base power offset for positive power
    if (power > 0){
      power += 0.08;
    }
    
    power = constrain(power, -1.0, 1.0);
    esc.writeMicroseconds(minPulseWidth + power * (maxPulseWidth - minPulseWidth));

    // Print debugging information
    Serial.print("Vel:");
    Serial.print(currentVelocity);
    Serial.print(",Target:");
    Serial.print(targetVelocity);
    Serial.print(",Pwr:");
    Serial.print(power);
    Serial.print(",Dist:");
    Serial.print(cumulativeDistance);
    Serial.print(",Tgt:");
    Serial.println(targetDist);

    // Update previousAngle for the next loop iteration
    previousAngle = currentAngle;
  }
}
