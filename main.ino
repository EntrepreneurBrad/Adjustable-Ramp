#include <Servo.h>
#include <IRremote.h>

// Define RGB LED pins
const int redPin = 8;
const int greenPin = 9;
const int bluePin = 10;

// Define the button and servo pin
const int buttonPin = 13;
const int servoPin = 6;

// Define the IR sensor pin
const int irSensorPin = 7;

// Variable to store the button state and IR sensor state
int buttonState = 0;
int irSensorState = 0;

// Include NewPing Library
#include "NewPing.h"

// Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define TRIGGER_PIN 11
#define ECHO_PIN 12

// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400	

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Create a Servo object
Servo servoMotor;

// Create an IRrecv object to receive IR signals
IRrecv irReceiver(irSensorPin);

// Create a decode_results object to store the received IR code
decode_results irResults;

void setup() {
  // Set RGB LED pins as OUTPUT
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize RGB LED to OFF (all colors off)
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);

  // Set button pin as INPUT_PULLUP (internal pull-up resistor enabled)
  pinMode(buttonPin, INPUT_PULLUP);

  // Start the IR receiver
  irReceiver.enableIRIn();

  // Attach the servo to the servoPin
  servoMotor.attach(servoPin);

  // Initialize Serial communication
  Serial.begin(9600);
}

void LEDRampStationary() { // Makes the LEDs Green when the ramp is stationary
  digitalWrite(redPin, HIGH);    // Turn off the red channel
  digitalWrite(greenPin, LOW);   // Turn on the green channel
  digitalWrite(bluePin, HIGH);   // Turn off the blue channel
}

void LEDRampMotion() { // Makes the LEDs Red when the ramp is stationary
  if (digitalRead(redPin) == LOW) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, HIGH);
  } else {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, HIGH);
  }
}

bool IRButtonPressed() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Check if the button is pressed (LOW) and the IR sensor detects an IR signal (LOW)
  if (buttonState == LOW && (irReceiver.decode(&irResults))) {
    irReceiver.resume();
    return true;
  }
  return false;
}

void RampMovement() {
  // read ultrasonic sensor and continue to until true is read
  while (!UltrasonicSensor()) {
    // run ramp motion down
    RampDownMotion();

    // run and wait for IRButtonPressed until true is returned
    while (!IRButtonPressed()) {
      // Wait for the button to be pressed
      delay(100);
    }

    // run ramp motion up
    RampUpMotion();
  }
}

void RampDownMotion() {
  // loop
  for (int angle = 180; angle >= 0; angle -= 10) {
    LEDRampMotion();
    servoMotor.write(angle);
    delay(100);
  }
}

void RampUpMotion() {
  // loop
  for (int angle = 0; angle <= 180; angle += 10) {
    LEDRampMotion();
    servoMotor.write(angle);
    delay(100);
  }
}

bool UltrasonicSensor() {
  // Read the distance from the ultrasonic sensor
  unsigned int distance = sonar.ping_cm();
  // Set the desired distance threshold (in centimeters)
  unsigned int thresholdDistance = 5;
  // Return true if the distance is less than the threshold, otherwise return false
  return (distance < thresholdDistance);
}

void loop() {
  LEDRampStationary();

  // run IRButtonPressed continuously until it is returned as True
  while (!IRButtonPressed()) {
    // Wait for the button to be pressed
    delay(100);
  }

  // if true is returned continue:

  // get servo angle to 180 to begin
  servoMotor.write(180);
  delay(500); // Wait for the servo to move to the initial position

  // run rampmovement and continue to run it
  RampMovement();
}
