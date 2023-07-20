#include <AccelStepper.h>

// Define stepper motor pins
const int DIR_PIN =43;
const int STEP_PIN = 45;

// Create AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN,41);

int angle = 0;
int temp_angle = 0;
int steps = 0;
int step_angle=0;

void setup() {
  // Set maximum speed and acceleration
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(5000);

  // Set initial position to zero
//  stepper.setCurrentPosition(stepper.move(steps));

  // Set pin modes for step and direction pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if there is serial data available
  if (Serial.available() > 0) {
    // Read the incoming angle value
    angle = Serial.parseInt();
    step_angle=angle-temp_angle;
    // Calculate the number of steps required to move to the new angle
    steps = (step_angle ) * 3200 / 360;
    if (angle != 0) {
      if (steps < 0) {
        // Move the motor clockwise
        digitalWrite(DIR_PIN, HIGH);
        stepper.move(steps);
        while (stepper.distanceToGo() != 0) {
          stepper.run();
        }
      }
       if (steps > 0) {
        // Move the motor anticlockwise
        digitalWrite(DIR_PIN, LOW);
        stepper.move(steps);
        while (stepper.distanceToGo() != 0) {
          stepper.run();
        }
        temp_angle = angle;
      }
      // Update the current angle
      temp_angle = angle;
      // Print the current angle to the serial monitor
      Serial.print("Angle: ");
      Serial.println(angle);
    }
  }
}
