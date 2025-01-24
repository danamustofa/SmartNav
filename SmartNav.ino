#include <Wire.h>
#include <NewPing.h>
#include <L298N.h>

// Motor driver pins
const int ENA = 22; // Motor A (left)
const int IN1 = 19;
const int IN2 = 18;
const int IN3 = 17;
const int IN4 = 16;
const int ENB = 23; // Motor B (right)

L298N motorLeft(ENA, IN1, IN2); // Motor A setup
L298N motorRight(ENB, IN3, IN4); // Motor B setup

int speed = 200; // Motor DC speed

// Ultrasonic sensor pins and max distance
#define TRIGGER_PIN 33
#define ECHO_PIN 32
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup

// Movement constants
#define FORWARD 1
#define BACKWARD 2
#define RIGHT 3
#define LEFT 4
#define STOP 0

// PID constants
double kp = 2.0;
double ki = 0.5;
double kd = 1.0;
double setPoint = 10.0; // Desired distance in cm
double input, output;
double lastInput = 0.0;
double integral = 0.0;

volatile bool objectDetected = false; // Flag to indicate object detection

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor with baud rate 9600

  // Attach interrupt to ECHO_PIN for object detection
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), detectObject, CHANGE);
}

void loop() {
  input = sonar.ping_cm(); // Get distance measurement in centimeters
  Serial.print("Distance: ");
  Serial.print(input);
  Serial.println(" cm");

  if (input > 0 && input < 10) {
    Serial.println("Object detected within 10 cm! Activating PID control.");
    pidCompute();
    moveCar(BACKWARD);
  } else {
    if (objectDetected) {
      Serial.println("Object no longer detected. Stopping.");
      moveCar(STOP);
      objectDetected = false;
    } else {
      moveCar(FORWARD);
    }
  }

  // Send data to Serial Plotter
  Serial.print("Distance: ");
  Serial.print(input);
  Serial.print(", PID Output: ");
  Serial.println(output);

  delay(100); // Short delay for demonstration
}

void detectObject() {
  // Function to handle object detection interrupt
  objectDetected = true;
}

void pidCompute() {
  double error = setPoint - input;
  integral += (error * 0.1); // Integral term (with a time step of 0.1 seconds)
  double derivative = (input - lastInput) / 0.1; // Derivative term
  output = kp * error + ki * integral - kd * derivative;
  lastInput = input;

  // Clamp output to valid motor speed range
  output = constrain(output, -255, 255);
  speed = abs(output);

  Serial.print("PID output: ");
  Serial.println(output);
}

void moveCar(int inputValue) {
  switch (inputValue) {
    case BACKWARD:
      Serial.println("Backward");
      motorLeft.setSpeed(speed);
      motorRight.setSpeed(speed);
      motorLeft.backward();
      motorRight.backward();
      break;

    case FORWARD:
      Serial.println("Forward");
      motorLeft.setSpeed(speed);
      motorRight.setSpeed(speed);
      motorLeft.forward();
      motorRight.forward();
      break;

    case RIGHT:
      Serial.println("Right");
      motorLeft.setSpeed(speed);
      motorRight.setSpeed(speed);
      motorLeft.backward();
      motorRight.forward();
      break;

    case LEFT:
      Serial.println("Left");
      motorLeft.setSpeed(speed);
      motorRight.setSpeed(speed);
      motorLeft.forward();
      motorRight.backward();
      break;

    case STOP:
      Serial.println("Stop");
      motorLeft.setSpeed(0);
      motorRight.setSpeed(0);
      motorLeft.stop();
      motorRight.stop();
      break;
  }
}
