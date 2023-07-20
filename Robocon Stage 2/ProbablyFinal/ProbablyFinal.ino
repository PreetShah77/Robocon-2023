#include <AccelStepper.h>
#include <Servo.h>
#include <FastLED.h>
#define NUM_LEDS 26
#define NUM_LED 28
CRGB led[84];
CRGB leds[26];
Servo esc1;
Servo esc2;
int pwm;
int rightpwm;
#define M1_pwm 2
#define M1_DIR 22
#define M2_pwm 3
#define M2_DIR 24
#define M3_pwm 4
#define M3_DIR 26
#define M4_pwm 5
#define M4_DIR 28
#define pick1_C 40
#define pick1_CC 38
#define pick2_C 44
#define pick2_CC 42
#define Shooting_1_Limit_Switch A1
#define Shooting_2_Limit_Switch A2
#define Shooting_1_Sliding_Motor_C 46
#define Shooting_1_Sliding_Motor_CC 48
#define Shooting_2_Sliding_Motor_C 50
#define Shooting_2_Sliding_Motor_CC 52
int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
bool temp4 = true;
bool temp5 = true;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
int temp = 0;
char button_value[0];
unsigned int l_value = 0;
unsigned int r_value = 0;
signed int l_y_value = 0;
signed int r_x_value = 0;
boolean lay = false;
int layout = 1;
int i, j, k;
int checkangle = 0;
int stepsis;
int robot_speed = 150;
int grab1 = 30;
int grab2 = 36;
int shoot1 = 34;
int shoot2 = 32;
const int stepPin4 = 45; //step 4 upper
const int dirPin4 = 43;
const int stepPin2 = 33; //step 2 upper
const int dirPin2 = 31;
const int stepPin1 = 27;  // step1 down
const int dirPin1 = 25;
const int stepPin3 = 39; //step3 down
const int dirPin3 = 37;
long zero[2] = {169, 169};
int pnue = 1;
//AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1, 23);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2, 29);
//AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3, 35);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4, 41);
long double pitcherror = 0;
//long double leftyaw[9] = {1, -25, 25}; // down
long double leftpitch[9] = {1 + pitcherror, 15 + pitcherror, 15 + pitcherror,25 + pitcherror}; //up
//long double rightyaw[9] = {1, -25, 25}; //down
long double rightpitch[9] = {-1 - pitcherror, -15 - pitcherror, -20 - pitcherror,-25 - pitcherror};
int a, b, c, d;
bool pitch1;
bool pitch2;
bool stepcheck;
int section = NUM_LED / 2;
long angle1, angle2, angle3, angle4;
void setup() {
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
  pinMode(grab1, OUTPUT);
  pinMode(grab2, OUTPUT);
  pinMode(shoot1, OUTPUT);
  pinMode(shoot2, OUTPUT);
  pinMode(pick1_C, OUTPUT);
  pinMode(pick1_CC, OUTPUT);
  pinMode(pick2_C, OUTPUT);
  pinMode(pick2_CC, OUTPUT);
  pinMode(Shooting_1_Limit_Switch, INPUT);
  pinMode(Shooting_2_Limit_Switch, INPUT);
  pinMode(Shooting_1_Sliding_Motor_C, OUTPUT);
  pinMode(Shooting_1_Sliding_Motor_CC, OUTPUT);
  pinMode(Shooting_2_Sliding_Motor_C, OUTPUT);
  pinMode(Shooting_2_Sliding_Motor_CC, OUTPUT);
//  stepper1.setMaxSpeed(2000);
//  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(10000);
  stepper2.setAcceleration(10000);
//  stepper3.setMaxSpeed(2000);
//  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(10000);
  stepper4.setAcceleration(10000);
  FastLED.addLeds<WS2811, 6, GRB>(leds, 26).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2811, 9, GRB>(led, 28).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(255);
  fill_solid(leds, 26, CRGB::Yellow);
  fill_solid(led, 28, CRGB::Yellow);
  FastLED.show();
  esc1.attach(8);
  esc2.attach(7);
  esc1.write(30);
  esc2.write(30);
  delay(1000);

}

void loop() {
  ps4_data();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
  if (button_value[0] != 'O') {
    lay = true;
  }
  if (button_value[0] == 'O' && lay == true) {
    layout++;
    if (layout == 4) {
      layout = 1;
    }
    lay = false;
  }
  if (layout == 1) {
    if (temp == 1)
    {
      int green = 255;
      Serial1.print('<');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print(green);
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('>');
      Serial1.println();
      fill_solid(leds, 26, CRGB::Blue);
      fill_solid(led, 28, CRGB::Blue);
      FastLED.show();
      temp = 2;
    }
    else;
    if(button_value[0]=='K'){
      digitalWrite(shoot1, HIGH);
      digitalWrite(shoot2, HIGH);
    }
    else if(button_value[0]=='M'){
      digitalWrite(shoot1, LOW);
      digitalWrite(shoot2, LOW);
    }
    else;
    movement();

      if (button_value[0] == 'E') {
      digitalWrite(pick2_C, HIGH);
      digitalWrite(pick2_CC, LOW);
    }
    else if (button_value[0] == 'G') {
      digitalWrite(pick2_C, LOW);
      digitalWrite(pick2_CC, HIGH);
    }
    else {
      digitalWrite(pick2_C, LOW);
      digitalWrite(pick2_CC, LOW);
    }

    if (button_value[0] == 'A') {
      digitalWrite(pick1_C, HIGH);
      digitalWrite(pick1_CC, LOW);
    }
    else if (button_value[0] == 'C') {
      digitalWrite(pick1_C, LOW);
      digitalWrite(pick1_CC, HIGH);
    }
    else {
      digitalWrite(pick1_C, LOW);
      digitalWrite(pick1_CC, LOW);
    }

    if (button_value[0] == 'F')      digitalWrite(grab1, HIGH);
    else if (button_value[0] == 'H') digitalWrite(grab1, LOW);

    if (button_value[0] == 'B')      digitalWrite(grab2, HIGH);
    else if (button_value[0] == 'D') digitalWrite(grab2, LOW);

  }
  if (layout == 2) {
    if (temp == 2)
    {
      int red = 255;
      Serial1.print('<');
      Serial1.print(red);
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('>');
      Serial1.println();
      fill_solid(led, 28, CRGB::Red);
      fill_solid(leds, 26, CRGB::Red);
      FastLED.show();
      temp = 3;
    }
    movement();

    
    if (button_value[0] == 'K') {
      esc1.write(120);
    }
    else    if (button_value[0] == 'L') {
      esc1.write(120);
    }
    else if (button_value[0] == 'M') {
      esc1.write(100);
    }
        else if (button_value[0] == 'D') {
      esc1.write(100);
    }
    else;
    

    if (button_value[0] == 'I' || button_value[0] == 'J') digitalWrite(shoot1, HIGH);
    else digitalWrite(shoot1, LOW);

    if (button_value[0] == 'E') {
      angle2 = leftpitch[0];
      stepangle2(angle2);
    }
    else if (button_value[0] == 'F') {
      angle2 = leftpitch[1];
      stepangle2(angle2);
    }
    
    else if (button_value[0] == 'G') {
      angle2 = leftpitch[2];
      stepangle2(angle2);
    }
    else if (button_value[0] == 'H') {
      angle2 = leftpitch[3];
      stepangle2(angle2);
    }
   else if (button_value[0] == 'C') {
      if (checkangle == 0) {
        angle2++;
        pitcherror++;
        stepangle2(angle2);
        checkangle = 1;
      }
      else;
    }
   
    else if (button_value[0] == 'A') {
      if (checkangle == 0) {
        angle2--;
        pitcherror--;
        stepangle2(angle2);
        checkangle = 1;
      }
      else;
    }
    else {
      checkangle = 0;
    }
  }
  if (layout == 3) {
    if (temp == 3)
    {
      int blue = 255;
      Serial1.print('<');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print(blue);
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('0');
      Serial1.print(',');
      Serial1.print('>');
      Serial1.println();
      fill_solid(led, 28, CRGB::Green);
      fill_solid(leds, 26, CRGB::Green);
      FastLED.show();
      temp = 1;
    }
    movement();
    
    if (button_value[0] == 'K') {
      esc2.write(120);
    }
    else if (button_value[0] == 'L') {
      esc2.write(120);
    }
    else if (button_value[0] == 'M') {
      esc2.write(100);
    }
        else if (button_value[0] == 'D') {
      esc2.write(100);
    }
    else;
    
    if (button_value[0] == 'I' || button_value[0] == 'J') digitalWrite(shoot2, HIGH);
    else digitalWrite(shoot2, LOW);

    if (button_value[0] == 'E') {
      angle4 = rightpitch[0];
      stepangle4(angle4);
    }
    else if (button_value[0] == 'F') {
      angle4 = rightpitch[1];
      stepangle4(angle4);
    }
    else if (button_value[0] == 'G') {
      angle4 = rightpitch[2];
      stepangle4(angle4);
    }
   else if (button_value[0] == 'H') {
      angle4 = rightpitch[3];
      stepangle4(angle4);
    }
      else if (button_value[0] == 'A') {
      if (checkangle == 0) {
        angle4++;
        pitcherror++;
        stepangle4(angle4);
        checkangle = 1;
      }
      else;
      }
    else if (button_value[0] == 'C') {
      if (checkangle == 0) {
        angle4--;
        pitcherror--;
        stepangle4(angle4);
        checkangle = 1;
      }
      else;
    }
    else {
      checkangle = 0;
    }
  }
}
void movement() {
  f_pwm = map(l_y_value, 10, 150, 10, robot_speed);
  b_pwm = map(l_y_value, -10, -150, 10, robot_speed);
  l_pwm = map(r_x_value, 10, 150, 10, robot_speed);
  r_pwm = map(r_x_value, -10, -150, 10, robot_speed);
  rr_pwm = map(r_value, 0, 255, 10, 80);
  lr_pwm = map(l_value, 0, 255, 10, 80);
  if (l_value > 10) Left_rotate(lr_pwm); //l1 button
  else if (r_value > 10) Right_rotate(rr_pwm); //r1 button
  else if (r_x_value < -10) Right(r_pwm); //analog right hat
  else if (r_x_value > 10) Left(l_pwm); // analog right hat
  else if (l_y_value > 10) Forward(f_pwm); // analog left hat
  else if (l_y_value < -10) Backward(b_pwm); // analog left hat
  else Hold();
}
