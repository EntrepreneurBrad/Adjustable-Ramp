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
  digitalWrite(redPin, LOW);    // Turn off the red channel
  digitalWrite(greenPin, HIGH);   // Turn on the green channel
  digitalWrite(bluePin, LOW);   // Turn off the blue channel
}

void LEDRampMotion() { // Makes the LEDs Red when the ramp is stationary
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
}

bool IRButtonPressed() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Check if the button is pressed (LOW) and the IR sensor detects an IR signal (LOW)
  if (buttonState == LOW && (irReceiver.decode(&irResults))) {
    irReceiver.resume();
    delay(500);
    return true;
  }
  return false;
}

void RampDownMotion() {
  LEDRampMotion();
    servoMotor.write(0);
}

void RampUpMotion() {
  LEDRampMotion();
  servoMotor.write(180);
}

void RampMovement() {
  while (true) {
    Serial.println("loop");

    // Read ultrasonic sensor and continue until true is read
    if (IRButtonPressed()) {
      Serial.println("IR button pressed. Ramping down motion.");
      RampDownMotion();
    }
    else {
      LEDRampStationary();
      Serial.println("LED ramp stationary.");
    }

    if (IRButtonPressed()) {
      Serial.println("IR button pressed. Ramping up.");
      RampUpMotion();
    }
    else {
      LEDRampStationary();
      Serial.println("LED ramp stationary.");
    }
  }
}



void loop() {
  LEDRampMotion();
  servoMotor.write(180);
  LEDRampStationary();
  RampMovement();
}
