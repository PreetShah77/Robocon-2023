#include "PS4.h"
#include "Drive.h"
#include <Servo.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <math.h>
//pid
Servo myServo;
Servo esc1;
Servo esc2;
double distance;
double strength;
int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
int correction_factorright=0;
int correction_factorleft=0;
int robot_speed = 255;
bool pidset = false;
bool setx = true;
bool sety = false;
bool checksum = false;
bool receiveComplete1 = false;
double set_posx = 42;
double outputx;
double kpx = 10;
double kdx = 0.9;
double kix = 0.7;
double set_posy = 52;
double outputy;
double kpy = 10;
double kdy = 0.9;
double kiy = 0.7;
double set_posd = 0;
double outputd;
double kpd = 5;
double kdd = 0.1;
double kid = 0.3;
int pwm;
double deg = 0;
double currenttime;
double previoustime;
double temptime;
double looptime;
double intervaldeg = 3000;
double intervalx = 5000;
double intervaly = 7000;
int i;
PID PIDX(&distance, &outputx, &set_posx, kpx, kdx, kix, DIRECT);
PID PIDY(&distance, &outputy, &set_posy, kpy, kdy, kiy, DIRECT);
PID PIDDEG(&deg, &outputd, &set_posd, kpd, kdd, kid, DIRECT);
//johnson
// int johnr1=36;
// int johnr2=34;
// int johnl1=32;
// int johnl2=30;
// int limit1=55;
// int limit2=56;
//stepper
long anglex[9] = {90,180,360,1};
long angley[9] = {90,180,360,1};
long anglex1[9] ={90,180,360,1};
long angley1[9] ={90,180,360,1};
const int stepPin1 = 45;
const int dirPin1 = 43;
const int stepPin3 = 33;
const int dirPin3 = 31;
const int stepPin2 = 27;
const int dirPin2 = 25;
long angle1, angle2, angle3, angle4;
const int stepPin4 = 39;
const int dirPin4= 37;
const int stepsPerRev = 3200;
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1,dirPin1,41);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2,dirPin2,23);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3,dirPin3,29);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4,dirPin4,35);

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial1.begin(9600);
  delay(20);
  Serial2.begin(115200);
  delay(20);
  Serial3.begin(115200);
  delay(20);

  myServo.attach(6);
  myServo.write(90);
  esc1.attach(9);
  esc2.attach(8);
  esc1.write(30);
  esc2.write(30);
  delay(1000)
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  pinMode(M3_pwm, OUTPUT);
  pinMode(M4_pwm, OUTPUT);
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-100, 100);
  PIDY.SetOutputLimits(-100, 100);
  PIDDEG.SetOutputLimits(-100, 100);
      //   digitalWrite(johnl1,LOW);
      // digitalWrite(johnl2,HIGH);
      // digitalWrite(johnr1,LOW);
      // digitalWrite(johnr2,HIGH);
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(1000);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(1000);
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(1000);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
}

void loop() {
  ps4_data();

  // if (Serial3.available()) {
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
  // int leftpickmax=digitalRead(limit1);
  // int rightpickmax=digitalRead(limit2);
  f_pwm = map(l_y_value, 10, 130, 10, robot_speed);
  b_pwm = map(l_y_value, -10, -130, 10, robot_speed);
  l_pwm = map(r_x_value, 10, 130, 10, robot_speed);
  r_pwm = map(r_x_value, -10, -130, 10, robot_speed);
  rr_pwm = map(r_value, 0, 255, 10, robot_speed);
  lr_pwm = map(l_value, 0, 255, 10, robot_speed);
  
  while (pidset == true) {
    Serial2.println('s');
    while (!receiveComplete1) {
      getTFminiData1(&distance, &strength, &receiveComplete1);
    }
    receiveComplete1 = false;
    // Serial.println(distance);

    while (Serial1.available()) {
      String x = Serial1.readStringUntil('\n');
      deg = x.toDouble();
      // Serial.println(deg);
    }
    currenttime = millis();
    if (i == 0) {
      temptime = currenttime;
      i = 1;
    }
    looptime = currenttime - temptime;
    
    Serial.print("   currenttime   ");
    Serial.print(currenttime);
    Serial.print("   temptime   ");
    Serial.print(temptime);
    Serial.print("    looptime   ");
    Serial.println(looptime);
    if (looptime <= intervaldeg) {
      setdegree();
      Serial.println("setting degree");
    } else {
      // Serial.println("X");
      if (looptime <= intervalx)  //&& deg>-1&&deg<1 )
      {
        myServo.write(90);
        Serial.println("setting x");
        setposx();
      } else {
        // Serial.println("Y");
        if (looptime <= intervaly)  //&& distance1>41&&distance1<43)
        {
          myServo.write(0);
          Serial.println("setting y");
          setposy();
        } else  //(distance1 >38&&distance1 <40 && distance2>39&&distance2 <41&& deg<0.5&&deg>-0.5)
        { 
          myServo.write(90);
          Serial2.println('q');
          pidset = false;
          // i = 0;
        }
      }
    }
  }
   if (l_value > 5) Left_rotate(lr_pwm);
  else if (r_value > 5) Right_rotate(rr_pwm);
  else if (r_x_value < -10) Right(r_pwm);
  else if (r_x_value > 10) Left(l_pwm);
  else if (l_y_value > 10) Forward(f_pwm);
  else if (l_y_value < -10) Backward(b_pwm);
  else if (button_value[0] == 'B') {
    pidset = true;
  } else if (button_value[0] == 'C') {
    esc1.write(150);
    esc2.write(150);
  }else if(button_value[0] == 'D'){
      // digitalWrite(johnl1,HIGH);
      // digitalWrite(johnl2,LOW);
      // digitalWrite(johnr1,HIGH);
      // digitalWrite(johnr2,LOW);
  }
  else if(button_value[0] == 'a'){
    angle1=anglex[0];
    angle3=anglex1[0];
    stepangle1(angle1);
    stepangle3(angle3);    
  } 
    else if(button_value[0] == 'b'){
    angle1=anglex[1];
    angle3=anglex1[1];
    stepangle1(angle1);
    stepangle3(angle3);
  } 
    else if(button_value[0] == 'c'){
    angle1=anglex[2];
    angle3=anglex1[2];
    stepangle1(angle1);
    stepangle3(angle3);
  } 
    else if(button_value[0] == 'd'){
    angle1=anglex[3];
    angle3=anglex1[3];
    stepangle1(angle1);
    stepangle3(angle3);
  } 
    else if(button_value[0] == 'e'){
    angle2=angley[0];
    angle4=angley1[0];
    stepangle4(angle4);
    stepangle2(angle2); 
  } 
    else if(button_value[0] == 'f'){
    angle2=angley[1];
    angle4=angley1[1];
    stepangle4(angle4);
    stepangle2(angle2);  

  } 
    else if(button_value[0] == 'g'){
    angle2=angley[2];
    angle4=angley1[2];
    stepangle4(angle4);
    stepangle2(angle2); 
  } 
    else if(button_value[0] == 'h'){
    angle2=angley[3];
    angle4=angley1[3];
    stepangle4(angle4);
    stepangle2(angle2); 
  } 
  else {
    Hold();
    // if(leftpickmax==0){
    //   digitalWrite(johnl1,LOW);
    //   digitalWrite(johnl2,LOW);
    // }
    // if(rightpickmax==0){
    //   digitalWrite(johnr1,LOW);
    //   digitalWrite(johnr2,LOW);
    // }
  }
  Serial.print('<');
  Serial.print(button_value);
  Serial.print(',');
  Serial.print(l_value);
  Serial.print(',');
  Serial.print(r_value);
  Serial.print(',');
  Serial.print(l_y_value);
  Serial.print(',');
  Serial.print(r_x_value);
  Serial.print('>');

  Serial.println();
}
// }