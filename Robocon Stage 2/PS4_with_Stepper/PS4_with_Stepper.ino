#include <SoftwareSerial.h>
#include <Servo.h>
#include <PID_v1.h>
#include <AccelStepper.h>
long anglex[9] = { 10, 20, 30, 40, 30, 20, 10, 90, 180 };
long angley[9] = { 10, 20, 30, 40, 30, 20, 10, 90, 180 };
Servo myServo;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;
char button_value[0];
unsigned int l_value = 0;
unsigned int r_value = 0;
signed int l_y_value = 0;
signed int r_x_value = 0;
int pwm;
const int stepPin1 = 45;
const int dirPin1 = 43;
long angle1, angle2;
const int stepPin2 = 39;
const int dirPin2 = 37;
const int stepsPerRev = 3200;
int dcm1 = 52;
int dcm2 = 50;
int dcm3 = 48;
int dcm4 = 46;
int pin1 = 44;
int pin2 = 42;
int pin3 = 40;
int pin4 = 38;
int jhonsonr1 = 36;
int jhonsonr2 = 34;
int jhonsonl1 = 32;
int jhonsonl2 = 30;
int limitswitchr1 = 55;
int limitswitchr2 = 56;
int limitswitchl1 = 57;
int limitswitchl2 = 58;
bool enter=true;
bool pidset;
int pin = 2;
bool servo = false;
int state;
int count = 0;
int count1 = 0;
double set_posx = 42;
double outputx;
double kpx = 9;
double kdx = 0.7;
double kix = 0.4;
double set_posy = 42;
double outputy;
double kpy = 9;
double kdy = 0.7;
double kiy = 0.4;
bool receiveComplete1 = false;
bool receiveComplete2 = false;
double distance1 = 0;
double strength1 = 0;
double distance2 = 0;
double strength2 = 0;
int servopos;
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1, 41);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2, 35);
PID PIDX(&distance1, &outputx, &set_posx, kpx, kdx, kix, DIRECT);
PID PIDY(&distance2, &outputy, &set_posy, kpy, kdy, kiy, DIRECT);
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  myServo.attach(9);
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-255, 255);
  PIDY.SetOutputLimits(-255, 255);
  attachInterrupt(digitalPinToInterrupt(pin), InterruptFunction, HIGH);
  pinMode(13, OUTPUT);
  pinMode(pin, INPUT);
  pinMode(dcm1, OUTPUT);
  pinMode(dcm2, OUTPUT);
  pinMode(dcm3, OUTPUT);
  pinMode(dcm4, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(jhonsonr1, OUTPUT);
  pinMode(jhonsonr2, OUTPUT);
  pinMode(jhonsonl1, OUTPUT);
  pinMode(jhonsonl2, OUTPUT);
  pinMode(limitswitchr1, INPUT);
  pinMode(limitswitchr2, INPUT);
  pinMode(limitswitchl1, INPUT);
  pinMode(limitswitchl2, INPUT);
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(100);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(100);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
}

void loop() {
  //label:
  ps4_data();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
  pidset = true;
  Serial.print(button_value);
  Serial.print(',');
  Serial.print(l_value);
  Serial.print(',');
  Serial.print(r_value);
  Serial.print(',');
  Serial.print(l_y_value);
  Serial.print(',');
  Serial.println(r_x_value);
  if (button_value[0] == 'A') {

     Serial.print("Hello");

    if (enter) {
      digitalWrite(pin1, HIGH);
      digitalWrite(dcm1, LOW);
      digitalWrite(dcm2, HIGH);
      delay(2000);
      digitalWrite(dcm1, LOW);
      digitalWrite(dcm2, LOW);
      delay(200);
      // digitalWrite(pin1, LOW);
    } else {
      digitalWrite(pin1, LOW);
      digitalWrite(dcm1, HIGH);
      digitalWrite(dcm2, LOW);
      delay(2000);
      digitalWrite(dcm1, LOW);
      digitalWrite(dcm2, LOW);
      // delay(200);
      digitalWrite(pin1, HIGH);
      enter=true;
    }
    count++;
  } 
  enter=false;
   if (button_value[0] == 'B') {
    Serial.println("Hello");
    if (count1 % 2 == 0) {
      digitalWrite(pin3, HIGH);
      digitalWrite(dcm3, LOW);
      digitalWrite(dcm4, HIGH);
      delay(2000);
      digitalWrite(dcm3, LOW);
      digitalWrite(dcm4, LOW);
      delay(200);
      // digitalWrite(pin2, LOW);
    } else {
      digitalWrite(pin3, LOW);
      digitalWrite(dcm3, HIGH);
      digitalWrite(dcm4, LOW);
      delay(2000);
      digitalWrite(dcm3, LOW);
      digitalWrite(dcm4, LOW);
      // delay(200);
      digitalWrite(pin3, HIGH);
    }
    count1++;
  } else if (button_value[0] == 'C') {
    // while (pidset) {
    //   while (!receiveComplete1) {
    //     getTFminiData1(&distance1, &strength1, &receiveComplete1);
    //   }
    //   receiveComplete1 = false;
    //   setposx();
    //   if (distance1 == abs(40)) {
    //     servo = true;
    //   }
    //   if (servo == true) {
    //     myServo.write(90);
    //   }
    //   servopos = myServo.read();
    //   if (distance1 == abs(40) && servopos == 90) {
    //     while (!receiveComplete2) {
    //       getTFminiData2(&distance2, &strength2, &receiveComplete2);
    //     }
    //     setposy();
    //     receiveComplete2 = false;
    //   }
    //   Serial.print("Distance1 = ");
    //   Serial.print(distance1);
    //   Serial.print("Distance2 = ");
    //   Serial.print(distance2);
    // }
  } else if (button_value[0] == 'a') {
    Serial.print("qwerty");
    angle1 = anglex[0];
    angle2 = angley[0];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'b') {
    angle1 = anglex[1];
    angle2 = angley[1];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'c') {
    angle1 = anglex[2];
    angle2 = angley[2];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'd') {
    angle1 = anglex[3];
    angle2 = angley[3];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'e') {
    angle1 = anglex[4];
    angle2 = angley[4];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'f') {
    angle1 = anglex[5];
    angle2 = angley[5];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'g') {
    angle1 = anglex[6];
    angle2 = angley[6];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'h') {
    angle1 = anglex[7];
    angle2 = angley[7];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (button_value[0] == 'I') {
    angle1 = anglex[8];
    angle2 = angley[8];
    stepangle1(angle1);
    stepangle2(angle2);
  } else if (l_value > 10) {
    int lrpwm = map(l_value, 10, 255, 0, 150);
    Left_rotate(lrpwm);
  } else if (r_value > 10) {
    int rrpwm = map(r_value, 10, 255, 0, 150);
    Right_rotate(rrpwm);
  } else if (l_y_value < -10) {
    int bpwm = map(l_y_value, -10, -127, 0, 150);
    Backward(bpwm);
  } else if (l_y_value > 10) {
    int fpwm = map(l_y_value, 10, 127, 0, 150);
    Forward(fpwm);
  } else if (r_x_value > 10) {
    int rpwm = map(r_x_value, 10, 127, 0, 150);
    Right(rpwm);
  } else if (r_x_value < -10) {
    int lpwm = map(r_x_value, -10, -127, 0, 150);
    Left(lpwm);
  } else {
    Hold();
  }
}
void InterruptFunction() {
  //pidset=false;
  // digitalWrite(pin,LOW);
  // loop();
  digitalWrite(13, HIGH);
  Hold();
  pidset = false;
}
