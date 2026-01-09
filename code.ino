#include <Servo.h>
#include <Wire.h>
#include "MT6701.h"
#include "ArduPID.h"

// Servo and Encoder
Servo esc;  // create servo object to control the ESC
Servo steeringServo;  // steering servo on pin 9
MT6701 encoder;

// PID Controller
ArduPID myController;

// Pin definitions
int pwmPin = 10;            // ESC input connected to digital pin 10
int steeringServoPin = 9;   // Steering servo on digital pin 9
const int button1 = 7;      // Button to toggle laser
const int button2 = 4;      // Button to start/stop driving
const int laser = 8;        // Laser control pin

// Pulse width limits for ESC
int minPulseWidth = 1500;   // Minimum pulse width (corresponding to minimum speed)
int maxPulseWidth = 2000;   // Maximum pulse width (corresponding to maximum speed)

// Adjustable variables
const float wheelDiameterInches = 2.875;                                      // Wheel diameter in inches
const float gearRatio = 1.2;                                                  // Gear ratio (encoder to wheel)
const float wheelCircumference = wheelDiameterInches * 2.54 * 3.14159265359;  // Circumference of the wheel in centimeters
const float targetDist = 888;  // target distance in centimeters
const float overshoot = 0.0;  // overshoot compensation in centimeters
const float accelerationDist = 180.0;  // distance over which acceleration occurs in centimeters
const float decelerationDist = 300.0;  // distance over which deceleration occurs in centimeters

// PID variables
double Setpoint, Input, Output;

// State variables
float previousAngle = 0.0;
float cumulativeDistance = 0.0;
float power = 0.0;
bool driving = false;  // State variable to control driving
bool PIDInitialize = false;  // Track whether PID has been initialized
bool decelerating = false;
bool laserOn = false;  // Laser state
bool encoderInitialized = false;

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
  steeringServo.write(90);  // Set to middle position immediately
  
  esc.attach(pwmPin);
  esc.writeMicroseconds(1500);  // Initialize ESC with 1ms pulse width
  
  pinMode(button1, INPUT_PULLUP);  // Use the internal pull-up resistor
  pinMode(button2, INPUT_PULLUP);  // Use the internal pull-up resistor
  pinMode(laser, OUTPUT);

  // Initialize ArduPID
  double Kp = 0.002, Ki = 0.0, Kd = 0.0;
  myController.begin(&Input, &Output, &Setpoint, Kp, Ki, Kd);
  myController.setOutputLimits(-1.0, 1.0);
  // myController.setBias(0.5);
  // myController.setWindUpLimits(-10, 10);
  // myController.start();
}

void loop() {
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

    // Calculate power based on distance traveled and target distance
    if (cumulativeDistance < accelerationDist) {
      // Linear acceleration
      power = cumulativeDistance / accelerationDist + 0.08;
    } else if (cumulativeDistance < targetDist - decelerationDist) {
      power = 1;
    } else {
      PIDInitialize = true;
    }

    if (PIDInitialize) {
      Setpoint = targetDist;
      myController.start();
      PIDInitialize = false;
      decelerating = true;
    }

    if (decelerating) {
      // Set PID input
      Input = cumulativeDistance;
      // Compute PID output
      myController.compute();
      power = Output;
    }

    if (power > 0){
      power += 0.08;
    }
    power = constrain(power, -1.0, 1.0);
    esc.writeMicroseconds(minPulseWidth + power * (maxPulseWidth - minPulseWidth));

    // Print the current angle and cumulative distance for debugging
    Serial.print(power);
    Serial.print(",");
    Serial.print(cumulativeDistance);
    Serial.print(",");
    Serial.println(targetDist);

    // Update previousAngle for the next loop iteration
    previousAngle = currentAngle;
  }
}
