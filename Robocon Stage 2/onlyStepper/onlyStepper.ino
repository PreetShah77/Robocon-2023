// Include the required libraries
#include <AccelStepper.h>

// Define the motor pins
#define STEP_PIN 5
#define DIR_PIN 7

// Define the motor steps per revolution (depends on your motor)
#define STEPS_PER_REVOLUTION 200

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Set the maximum speed and acceleration of the stepper motor
  stepper.setMaxSpeed(1000);    // Adjust this value as per your motor
  stepper.setAcceleration(500); // Adjust this value as per your motor

  // Set the initial position of the stepper motor (optional)
  stepper.setCurrentPosition(0);
}

void loop() {
  // Set the desired angle to move (in degrees)
  float desiredAngle = Serial.parseInt();

  // Calculate the number of steps required to move the desired angle
  int stepsToMove = (desiredAngle / 360.0) * STEPS_PER_REVOLUTION;

  // Move the stepper motor to the desired position
  stepper.moveTo(stepsToMove);

  // Run the stepper motor until it reaches the desired position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Wait for a moment before moving to the next position
  delay(1000);
}
