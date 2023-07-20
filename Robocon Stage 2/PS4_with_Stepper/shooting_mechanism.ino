long temp_angle1=0;
long steps1 = 0;
long step_angle1 = 0;
long temp_angle2=0;
long steps2 = 0;
long step_angle2 = 0;


void stepangle1(int angle1){
    Serial.print("ManthanLodu");
    step_angle1=angle1-temp_angle1;
    // Calculate the number of steps required to move to the new angle
    steps1 = (step_angle1 ) * 3200 / 360;
    if (angle1 != 0) {
      if (steps1 < 0) {
        // Move the motor clockwise
        digitalWrite(dirPin1, HIGH);
        stepper1.move(steps1);
        while (stepper1.distanceToGo() != 0) {
          stepper1.run();
        }
      }
      else if (steps1 > 0) {
        // Move the motor anticlockwise
        digitalWrite(dirPin1, LOW);
        stepper1.move(steps1);
        while (stepper1.distanceToGo() != 0) {
          stepper1.run();
        }
        temp_angle1 = angle1;
      }
      // Update the current angle
      temp_angle1 = angle1;
      // Print the current angle to the serial monitor
      Serial.print("Angle1: ");
      Serial.println(angle1);
}
}
void stepangle2(int angle2){
      step_angle2=angle2-temp_angle2;
    // Calculate the number of steps required to move to the new angle
    steps2 = (step_angle2 ) * 3200 / 360;
    if (angle2 != 0) {
      if (steps2 < 0) {
        // Move the motor clockwise
        digitalWrite(dirPin2, HIGH);
        stepper2.move(steps2);
        while (stepper2.distanceToGo() != 0) {
          stepper2.run();
        }
      }
      else if (steps2 > 0) {
        // Move the motor anticlockwise
        digitalWrite(dirPin2, LOW);
        stepper2.move(steps2);
        while (stepper2.distanceToGo() != 0) {
          stepper2.run();
        }
        temp_angle2 = angle2;
      }
      // Update the current angle
      temp_angle2 = angle2;
      // Print the current angle to the serial monitor
      Serial.print("Angle2: ");
      Serial.println(angle2);

    }
}