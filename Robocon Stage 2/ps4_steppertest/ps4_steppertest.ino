#include <AccelStepper.h>
const int stepPin4 = 45; //step 4 upper
const int dirPin4 = 43;
const int stepPin3 = 39; //step3 down
const int dirPin3 = 37;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];   
boolean newData = false;
int angle3,angle4;

char button_value[0];
unsigned int l_value = 0;
unsigned int r_value = 0;
signed int l_y_value = 0;
signed int r_x_value = 0;

long temp_angle3 = 0;
long steps3 = 0;
long step_angle3 = 0;
long temp_angle4 = 0;
long steps4 = 0;
long step_angle4 = 0;

long anglex[9] = {-10, 1,1,1};   // down
long angley[9] = { 10, 1, 1, 1 };   //up
long anglex1[9] = { -10, 1, 1 };  //down
long angley1[9] = { 10, 1, 1, 1 };  //

AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3, 35);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4, 41);
void setup() {
  // put your setup code here, to run once:
    pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
    stepper3.setMaxSpeed(5000);
  stepper3.setAcceleration(5000);
  stepper4.setMaxSpeed(5000);
  stepper4.setAcceleration(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
    ps4_data();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
    if (button_value[0] == 'e') {
    angle3 = anglex1[0];
    angle4 = angley1[0];
    stepangle3(angle3);
    stepangle4(angle4);
  }
    else  if (button_value[0] == 'f') {
    angle3 = anglex1[1];
    angle4 = angley1[1];
    stepangle3(angle3);
    stepangle4(angle4);
  }
  
  
  
}
void stepangle3(int angle3) {
  step_angle3 = angle3 - temp_angle3;
  // Calculate the number of steps required to move to the new angle
  steps3 = (step_angle3)*60800 / 360;
  if (angle3 != 0) {
    if (steps3 < 0) {
      // Move the motor clockwise
      digitalWrite(dirPin3, HIGH);
      stepper3.move(steps3);
      while (stepper3.distanceToGo() != 0) {
        stepper3.run();
      }
    } else if (steps3 > 0) {
      // Move the motor anticlockwise
      digitalWrite(dirPin3, LOW);
      stepper3.move(steps3);
      while (stepper3.distanceToGo() != 0) {
        stepper3.run();
      }
      temp_angle3 = angle3;
    }
    // Update the current angle
    temp_angle3 = angle3;
    // Print the current angle to the serial monitor
    Serial.print("Angle3: ");
    Serial.println(angle3);
  }
}
//upper stepper 4
void stepangle4(int angle4) {
  step_angle4 = angle4 - temp_angle4;
  // Calculate the number of steps required to move to the new angle
  steps4 = (step_angle4)*60800 / 360;
  if (angle4 != 0) {
    if (steps4 < 0) {
      // Move the motor clockwise
      digitalWrite(dirPin4, HIGH);
      stepper4.move(steps4);
      while (stepper4.distanceToGo() != 0) {
        stepper4.run();
      }
    } else if (steps4 > 0) {
      // Move the motor anticlockwise
      digitalWrite(dirPin4, LOW);
      stepper4.move(steps4);
      while (stepper4.distanceToGo() != 0) {
        stepper4.run();
      }
      temp_angle4 = angle4;
    }
    // Update the current angle
    temp_angle4 = angle4;
    // Print the current angle to the serial monitor
    Serial.print("Angle4: ");
    Serial.println(angle4);
  }
}
void ps4_data() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial3.available() > 0 && newData == false) {
        rc = Serial3.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void split_data() {      
    char * strtokIndx;

    strtokIndx = strtok(tempChars,",");      
    strcpy(button_value, strtokIndx); 
 
    strtokIndx = strtok(NULL, ","); 
    l_value = atoi(strtokIndx);     

    strtokIndx = strtok(NULL, ",");
    r_value = atoi(strtokIndx);
         
    strtokIndx = strtok(NULL, ",");
    l_y_value = atoi(strtokIndx);    

    strtokIndx = strtok(NULL, ",");
    r_x_value = atoi(strtokIndx); 
}
