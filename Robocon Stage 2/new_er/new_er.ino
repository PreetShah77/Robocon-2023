#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <FastLED.h>
CRGB leds[26];
Servo esc1;
Servo esc2;
#define shoot1 30
#define shoot2 36
#define M1_pwm 2
#define M1_DIR 22
#define M2_pwm 3
#define M2_DIR 24
#define M3_pwm 4
#define M3_DIR 26
#define M4_pwm 5
#define M4_DIR 28
#define Shooting_1_Limit_Switch A3
#define Shooting_2_Limit_Switch A4
#define Shooting_1_Sliding_Motor_C 46
#define Shooting_1_Sliding_Motor_CC 48
#define Shooting_2_Sliding_Motor_C 50
#define Shooting_2_Sliding_Motor_CC 52
int pwm;
int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
int robot_speed = 50;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
bool temp = true;
char button_value[0];
unsigned int l_value = 0;
unsigned int r_value = 0;
signed int l_y_value = 0;
signed int r_x_value = 0;
boolean lay = false;
int layout = 1;
bool escc1,escc2;
int count1,count2;
const int stepPin1 = 27;  // step1 down
const int dirPin1 = 25;
const int stepPin3 = 39; //step3 down
const int dirPin3 = 37;
const int stepPin4 = 45; //step 4 upper
const int dirPin4 = 43;
const int stepPin2 = 33; //step 2 upper
const int dirPin2 = 31;
bool temp4 = true;
bool temp5 = true;
int positionsyaw;
int positionspitch;
long positions[1];
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1, 23);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin1, 20);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3, 35);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4, 41);
MultiStepper steppers1;
MultiStepper steppers2;
int pitcherror = 14;
long leftpitch[9] = {-23.42, -35.072354, -20.39, 0};
long rightpitch[9] = {-20.465, -23.714, -22.2458, -32.9648, 0};
int a,b,c,d;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  pinMode(M3_pwm, OUTPUT);
  pinMode(M4_pwm, OUTPUT);
  pinMode(shoot1, OUTPUT);
  pinMode(shoot2, OUTPUT);
  pinMode(Shooting_1_Limit_Switch,INPUT);
  pinMode(Shooting_2_Limit_Switch,INPUT);
  pinMode(Shooting_1_Sliding_Motor_C,OUTPUT);
  pinMode(Shooting_1_Sliding_Motor_CC,OUTPUT);
  pinMode(Shooting_2_Sliding_Motor_C,OUTPUT);
  pinMode(Shooting_2_Sliding_Motor_CC,OUTPUT);
stepper1.setMaxSpeed(5000);
  stepper2.setMaxSpeed(5000);
  stepper3.setMaxSpeed(5000);
  stepper4.setMaxSpeed(5000);
  steppers1.addStepper(stepper4);
  steppers1.addStepper(stepper3);
 steppers2.addStepper(stepper2);
  steppers2.addStepper(stepper1);
  esc1.attach(8);
  esc2.attach(7);
  esc1.write(30);
  esc2.write(30);
  delay(1000);
  FastLED.addLeds<WS2811, 6, GRB>(leds, 26).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(255);
  fill_solid(leds, 26, CRGB::Red);
  FastLED.show();
}

void loop() {
  // put your main code here, to run repeatedly:
    ps4_data();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
    movement();
    if(button_value[0]!='A') escc1=true;
    if(button_value[0]=='A' && escc1==true){ 
      count1++;
      if(count1==3){
        count1=1;
      }
      escc1=false;
    }

    if(button_value[0]!='E') escc2=true;
    if(button_value[0]=='E' && escc2==true){ 
      count2++;
      if(count2==3){
        count2=1;
      }
      escc2=false;
    }
    
  if(button_value[0] == 'K') Mechanism_Homing();
  else if(button_value[0] == 'M') Mechanism_Expand();
  else;
    
  if(count1==1){ esc2.write(150); }
  else if(count1==2){ esc2.write(30);  }
  else;
    if(count2==1){ esc1.write(150);  }
  else if(count2==2){ esc1.write(30);  }
  else;

  if(button_value[0] == 'I') digitalWrite(shoot1,HIGH);
  else digitalWrite(shoot1,LOW);

  if(button_value[0] == 'J') digitalWrite(shoot2,HIGH);
  else digitalWrite(shoot2,LOW);

   if (button_value[0] == 'B') {
   
      positions[0] = (leftpitch[0] * 60800) / 360;
      positions[1] = 0;
      steppers1.moveTo(positions);
      steppers1.runSpeedToPosition();
    }
    else if (button_value[0] == 'C') {
      positions[0] = (leftpitch[1] * 60800) / 360;
      positions[1] = 0;
      steppers1.moveTo(positions);
      steppers1.runSpeedToPosition();
    }
        else if (button_value[0] == 'D') {
      positions[0] = (leftpitch[2] * 60800) / 360;
      positions[1] = 0;
      steppers1.moveTo(positions);
      steppers1.runSpeedToPosition();
    }
    else;

       if (button_value[0] == 'F') {
      positions[0] = (rightpitch[1] * 60800) / 360;
      positions[1] = 0;
      steppers2.moveTo(positions);
      steppers2.runSpeedToPosition();
    }
    else if (button_value[0] == 'G') {
      positions[0] = (rightpitch[2] * 60800) / 360;
      positions[1] = 0;
      steppers2.moveTo(positions);
      steppers2.runSpeedToPosition();
    }
        else if (button_value[0] == 'H') {
      positions[0] = (rightpitch[3] * 60800) / 360;
      positions[1] = 0;
      steppers2.moveTo(positions);
      steppers2.runSpeedToPosition();
    }
    else;
}
void movement() {
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
  else Hold();
}
void Mechanism_Expand()
{
  if(temp5 == true)
  {
    fill_solid(leds,26,CRGB::Yellow);
    digitalWrite(Shooting_1_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_1_Sliding_Motor_CC, HIGH);
    digitalWrite(Shooting_2_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_CC, HIGH);
    delay(13000);
    digitalWrite(Shooting_1_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_1_Sliding_Motor_CC, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_CC, LOW);
    c = digitalRead(Shooting_1_Limit_Switch);
    while(c == 1){
      c = digitalRead(Shooting_1_Limit_Switch);
      digitalWrite(Shooting_1_Sliding_Motor_C, LOW);
      digitalWrite(Shooting_1_Sliding_Motor_CC, HIGH);
    }
    digitalWrite(Shooting_1_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_1_Sliding_Motor_CC, LOW);
    
    d = digitalRead(Shooting_2_Limit_Switch);
    while(d == 1){
      d = digitalRead(Shooting_2_Limit_Switch);
      digitalWrite(Shooting_2_Sliding_Motor_C, LOW);
      digitalWrite(Shooting_2_Sliding_Motor_CC, HIGH);
    }
    digitalWrite(Shooting_2_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_CC, LOW);
    fill_solid(leds,26,CRGB::Green);
    FastLED.show();
    temp5 = false;
  }
}

void Mechanism_Homing()
{
  if(temp4 == true)
  {
    fill_solid(leds,26,CRGB::Yellow);
    digitalWrite(Shooting_1_Sliding_Motor_C, HIGH);
    digitalWrite(Shooting_1_Sliding_Motor_CC, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_C, HIGH);
    digitalWrite(Shooting_2_Sliding_Motor_CC, LOW);
    delay(13000);
    digitalWrite(Shooting_1_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_1_Sliding_Motor_CC, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_CC, LOW);
    c = digitalRead(Shooting_1_Limit_Switch);
    while(c == 1){
      c = digitalRead(Shooting_1_Limit_Switch);
      digitalWrite(Shooting_1_Sliding_Motor_C, HIGH);
      digitalWrite(Shooting_1_Sliding_Motor_CC, LOW);
    }
    digitalWrite(Shooting_1_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_1_Sliding_Motor_CC, LOW);
    
    d = digitalRead(Shooting_2_Limit_Switch);
    while(d == 1){
      d = digitalRead(Shooting_2_Limit_Switch);
      digitalWrite(Shooting_2_Sliding_Motor_C, HIGH);
      digitalWrite(Shooting_2_Sliding_Motor_CC, LOW);
    }
    digitalWrite(Shooting_2_Sliding_Motor_C, LOW);
    digitalWrite(Shooting_2_Sliding_Motor_CC, LOW);
    fill_solid(leds,26,CRGB::Green);
    FastLED.show();
    temp4 = false;
  }
}
