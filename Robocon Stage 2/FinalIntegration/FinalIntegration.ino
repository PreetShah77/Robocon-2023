#include "PS4.h"
#include "Drive.h"
#include <Servo.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <math.h>
#include <FastLED.h>
CRGB leds[26];

int layout = 1;
//movement variables
int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
int correction_factorright = 0;
int correction_factorleft = 0;
int robot_speed = 255;
int pwm;
//esc
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
//pid
Servo myServo;
int countesc = 0;
double distance;
double strength;
double deg = 0;
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
double currenttime;
double previoustime;
double temptime;
double looptime;
double currenttimeexpand;
double previoustimeexpand;
double temptimeexpand;
double looptimeexpand;
double currenttimecontract;
double previoustimecontract;
double temptimecontract;
double looptimecontract;
double intervalexpand = 18000;
double intervalcontract = 18000;
double intervaldeg = 3000;
double intervalx = 5000;
double intervaly = 7000;
int i;
int j;
int k;
PID PIDX(&distance, &outputx, &set_posx, kpx, kdx, kix, DIRECT);
PID PIDY(&distance, &outputy, &set_posy, kpy, kdy, kiy, DIRECT);
PID PIDDEG(&deg, &outputd, &set_posd, kpd, kdd, kid, DIRECT);
//johnson
int johnr1 = 46;
int johnr2 = 48;
int johnl1 = 50;
int johnl2 = 52;
int limit1 = 55;
int limit2 = 56;
int limit3;
int limit4;
int limit5;
int limit6;
int limit7;
int leftpickmax = digitalRead(limit1);
int rightpickmax = digitalRead(limit2);
//stepper
long double leftyaw[9] = {-62, -1, -20, 1}; // down
long double leftpitch[9] = { 20, 23, 23, 32, 1}; //up
long double rightyaw[9] = {62, 1, 20, 1}; //down
long double rightpitch[9] = { 20, 23, 23, 32, 1}; //up
const int stepPin4 = 45; //step 4 upper
const int dirPin4 = 43;
const int stepPin2 = 33; //step 2 upper
const int dirPin2 = 31;
const int stepPin1 = 27;  // step1 down
const int dirPin1 = 25;
const int stepPin3 = 39; //step3 down
const int dirPin3 = 37;
const int stepsPerRev = 3200;
long angle1, angle2, angle3, angle4;
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1, 23);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2, 29);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3, 35);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4, 41);
boolean lay = false;
int lpick1 = 44;
int lpick2 = 42;
int rpick1 = 40;
int rpick2 = 38;
int pnu1 = 32;
int pnu2 = 34;
bool contract = false;
bool expand;
bool temp = true;
int shootpnu1 = 36;
int shootpnu2 = 30;  
void loop() {
  ps4_data();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
  // digitalRead(limitswitch1);
  // digitalRead(limitswitch2);
  int leftpickmax = digitalRead(limit3);
  int leftpickmin = digitalRead(limit4);
  int johnny1 = digitalRead(limit5);
  int johnny2 = digitalRead(limit6);
  // digitalRead(limit5);
  // digitalRead(limit6);
  // digitalRead(limit7);
  // digitalRead(limit8);
  // digitalRead(limit9);
  // digitalRead(limit10);
  if (button_value[0] != 'O') {
    lay = true;
  }
  if (button_value[0] == 'O' && lay == true) {
    layout++;
    if (layout == 3) {
      layout = 1;
    }
    lay = false;
  }
  if (layout == 1) {
    if (temp == true)
    {
      int red = 160;
      int green = 32;
      int blue = 240;
      Serial3.print('<');
      Serial3.print(red);
      Serial3.print(',');
      Serial3.print(blue);
      Serial3.print(',');
      Serial3.print(green);
      Serial3.print(',');
      Serial3.print('0');
      Serial3.print(',');
      Serial3.print('0');
      Serial3.print(',');
      Serial3.print('>');
      Serial3.println();
      temp = false;
    }
    f_pwm = map(l_y_value, 10, 130, 10, robot_speed);
    b_pwm = map(l_y_value, -10, -130, 10, robot_speed);
    l_pwm = map(r_x_value, 10, 130, 10, robot_speed);
    r_pwm = map(r_x_value, -10, -130, 10, robot_speed);
    rr_pwm = map(r_value, 0, 255, 10, robot_speed);
    lr_pwm = map(l_value, 0, 255, 10, robot_speed);
    if (l_value > 10) Left_rotate(lr_pwm); //l1 button
    else if (r_value > 10) Right_rotate(rr_pwm); //r1 button
    else if (r_x_value < -10) Right(r_pwm); //analog right hat
    else if (r_x_value > 10) Left(l_pwm); // analog right hat
    else if (l_y_value > 10) Forward(f_pwm); // analog left hat
    else if (l_y_value < -10) Backward(b_pwm); // analog left hat

    else if (button_value[0] == 'C') {
      digitalWrite(lpick1, HIGH);
      digitalWrite(lpick2, LOW);
    }
    else if (button_value[0] == 'A') {
      digitalWrite(lpick1, LOW);
      digitalWrite(lpick2, HIGH);
    }
    else if (button_value[0] == 'B') {
      digitalWrite(pnu1, HIGH);
    }
    else if (button_value[0] == 'D') {
      digitalWrite(pnu1, LOW);
    }
    else if (button_value[0] == 'G') {
      digitalWrite(rpick1, HIGH);
      digitalWrite(rpick2, LOW);
    }
    else if (button_value[0] == 'E') {
      digitalWrite(rpick1, LOW);
      digitalWrite(rpick2, HIGH);
    }
    else if (button_value[0] == 'F') {
      digitalWrite(pnu2, HIGH);
    }
    else if (button_value[0] == 'H') {
      digitalWrite(pnu2, LOW);
    }
    else if (button_value[0] == 'L') {
      expand = true;
      contract = false;
      pidset = true;
    }
    else Hold();
    //analog ps4 buttons mapping
  }
  if (layout == 2) {
    if (temp == false)
    {
      int red = 255;
      Serial3.print('<');
      Serial3.print(red);
      Serial3.print(',');
      Serial3.print('0');
      Serial3.print(',');
      Serial3.print('0');
      Serial3.print(',');
      Serial3.print('0');
      Serial3.print(',');
      Serial3.print('0');
      Serial3.print(',');
      Serial3.print('>');
      Serial3.println();
      temp = true;
    }

    if (button_value[0] == 'K') {
//      esc1.write(150);
//      esc2.write(150);
      esc3.write(150);
      esc4.write(150);
    }
    else if (button_value[0] == 'M') {
//      esc1.write(30);
//      esc2.write(30);
      esc3.write(30);
      esc4.write(30);
    }
    else;

    if (r_value > 50) {
      Serial.println("Hello");
      digitalWrite(shootpnu2, HIGH);
    }
    else {
      digitalWrite(shootpnu2, LOW);
    }
    if (l_value > 50) {
      digitalWrite(shootpnu1, HIGH);
    }
    else {
      digitalWrite(shootpnu1, LOW);
    }
    if (button_value[0] == 'A') {
      angle1 = leftyaw[0];
      angle2 = leftpitch[0];
      stepangle1(angle1);
      stepangle2(angle2);
    }
    //setting pole angle x mechanism
    else if (button_value[0] == 'B') {
      pwm = 0;
      angle1 = leftyaw[1];
      angle2 = leftpitch[1];
      stepangle1(1);
      stepangle2(1);
    }
    //setting pole angle x mechanism
    else if (button_value[0] == 'C') {
      pwm = 0;
      angle1 = leftyaw[2];
      angle2 = leftpitch[2];
      stepangle2(angle2);
      stepangle1(angle1);
  }
    //setting pole angle x mechanism
    else if (button_value[0] == 'D') {
      angle1 = leftyaw[1];
      angle2 = leftpitch[1];
       stepangle2(angle2);
      stepangle1(angle1);
       
    }
    else if (button_value[0] == 'I') {

      angle1 = leftyaw[3];
      angle2 = leftpitch[4];
            stepangle2(angle2);
      stepangle1(angle1);
  }

    //setting pole angle y mechanism
    else if (button_value[0] == 'E') {
      angle3 = rightyaw[0];
      angle4 = rightpitch[0];
      stepangle3(angle3);
      stepangle4(angle4);
    }
    //setting pole angle y mechanism
    else if (button_value[0] == 'F') {
      angle3 = rightyaw[1];
      angle4 = rightpitch[1];
      stepangle3(angle3);
      stepangle4(angle4);
    }
    //setting pole angle y mechanism
    else if (button_value[0] == 'G') {
      angle3 = rightyaw[2];
      angle4 = rightpitch[2];
      stepangle3(angle3);
      stepangle4(angle4);
    }
    //setting pole angle y mechanism
    else if (button_value[0] == 'H') {
      angle3 = rightyaw[3];
      angle4 = rightpitch[3];
      stepangle3(angle3);
      stepangle4(angle4);
    }
    else if (button_value[0] == 'J') {
      angle3 = rightyaw[3];
      angle4 = rightpitch[4];
      stepangle3(angle3);
      stepangle4(angle4);
    }
    else Hold();
  }

  //johnson motor loop
  while (expand == true) {
    currenttimeexpand = millis();
    if (j == 0) {
      temptimeexpand = currenttimeexpand;
      j = 1;
    }
    looptimeexpand = currenttimeexpand - temptimeexpand;
    if (looptimeexpand <= intervalexpand) {
      if (johnny1 == 1 && johnny2 == 1) {
        digitalWrite(johnl1, HIGH);
        digitalWrite(johnl2, LOW);
        digitalWrite(johnr1, LOW);
        digitalWrite(johnr2, HIGH);
      } else if (johnny1 == 0) {
        digitalWrite(johnl1, LOW);
        digitalWrite(johnl2, LOW);
      } else if (johnny2 == 0) {
        digitalWrite(johnr1, LOW);
        digitalWrite(johnr2, LOW);
      } else if (johnny1 == 0 && johnny2 == 0) {
        digitalWrite(johnl1, LOW);
        digitalWrite(johnl2, LOW);
        digitalWrite(johnr1, LOW);
        digitalWrite(johnr2, LOW);
      }
    }
  }
  while (contract == true) {
    currenttimecontract = millis();
    if (j == 0) {
      temptimecontract = currenttimecontract;
      j = 1;
    }
    looptimecontract = currenttimecontract - temptimecontract;
    if (looptimecontract <= intervalcontract) {
      if (johnny1 == 1 && johnny2 == 1) {
        digitalWrite(johnl1, LOW);
        digitalWrite(johnl2, HIGH);
        digitalWrite(johnr1, HIGH);
        digitalWrite(johnr2, LOW);
      } else if (johnny1 == 0) {
        digitalWrite(johnl1, LOW);
        digitalWrite(johnl2, LOW);
      } else if (johnny2 == 0) {
        digitalWrite(johnr1, LOW);
        digitalWrite(johnr2, LOW);
      } else if (johnny1 == 0 && johnny2 == 0) {
        digitalWrite(johnl1, LOW);
        digitalWrite(johnl2, LOW);
        digitalWrite(johnr1, LOW);
        digitalWrite(johnr2, LOW);
      }
    }
  }



  //Pid loop
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

      if (looptime <= intervalx)
      {
        myServo.write(90);
        Serial.println("setting x");
        setposx();
      } else {
        if (looptime <= intervaly)
        {
          myServo.write(0);
          Serial.println("setting y");
          setposy();
        } else {
          myServo.write(90);
          Serial2.println('q');
          pidset = false;
          //  myServo.write(90);
          // i = 0;
        }
      }
    }
  }
  //analog movement

  //setting pole angle x mechanism

  //else part for limit switches and hold() functoion;



  //printing ps4 values
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
