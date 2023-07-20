#include <Servo.h>
#include <PID_v1.h>
Servo esc1;
Servo esc2;
#define M1_pwm 2
#define M1_DIR 22
#define M2_pwm 3
#define M2_DIR 24
#define M3_pwm 4
#define M3_DIR 26
#define M4_pwm 5
#define M4_DIR 28
#define pick_c 47
#define pick_cc 49
#define grab 51
#define shoot 53
int pwm;
double distance1;
double strength1;
double distance2;
double strength2;
double deg = 0;
bool pidset = false;
bool setx = true;
bool sety = false;
bool checksum = false;
bool receiveComplete1 = false;
bool receiveComplete2 = false;
double set_posx = 121;
double outputx;
double kpx = 7; //9
double kdx = 0.7;
double kix = 0.2;
double set_posy = 21;
double outputy;
double kpy = 4;
double kdy = 0.8;
double kiy = 0.55;
double set_posd;
double outputd;
double kpd = 2.5; //4
double kdd = 0.2; //0.1
double kid = 0.1; //0.3
double currenttime;
double previoustime;
double temptime;
double looptime;
double intervaldeg = 3000;
double intervalx = 6000;
double intervaly = 8000;
double intervalyaw=5000;
int i,j,k;
int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
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
int robot_speed=255;
PID PIDX(&distance1, &outputx, &set_posx, kpx, kdx, kix, DIRECT);
PID PIDY(&distance2, &outputy, &set_posy, kpy, kdy, kiy, DIRECT);
PID PIDDEG(&deg, &outputd, &set_posd, kpd, kdd, kid, DIRECT);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  pinMode(M3_pwm, OUTPUT);
  pinMode(M4_pwm, OUTPUT);
  pinMode(pick_c,OUTPUT);
  pinMode(pick_cc,OUTPUT);
  pinMode(grab,OUTPUT);
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-100,100);
  PIDY.SetOutputLimits(-100,100);
  PIDDEG.SetOutputLimits(-100,100);
  esc1.attach(7);
  esc2.attach(8);
  esc1.write(30);
  esc2.write(30);
  delay(1000);
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
  if(button_value[0] == 'A'){ digitalWrite(pick_c,HIGH); digitalWrite(pick_cc,LOW);}
  else if(button_value[0] == 'C'){digitalWrite(pick_c,LOW); digitalWrite(pick_cc,LOW); }
  else digitalWrite(pick_c,LOW); digitalWrite(pick_cc,LOW);

  if(button_value[0] == 'B') digitalWrite(grab,HIGH);
  else if(button_value[0] == 'D') digitalWrite(grab,LOW);
  else;
  if(button_value[0] == 'L'){pidset=true; pid(pidset);}

  
  
  if(button_value[0] == 'k'){ esc1.write(150); esc2.write(150); }
  else if(button_value[0] == 'm'){ esc1.write(30); esc2.write(30); }
  else;

  
  
 
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
void movement(){
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
void getTFminiData1(double * distance, double * strength, boolean * complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  //  port->listen();
  if (Serial.available()) {
    rx[i] = Serial.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
void getTFminiData2(double * distance, double * strength, boolean * complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  //  port->listen();
  if (Serial2.available()) {
    rx[i] = Serial2.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
void pid(bool pidset){
  while (pidset == true) {
      while (!receiveComplete1) {
        getTFminiData1(&distance1, &strength1, &receiveComplete1);
      }
      receiveComplete1 = false;

      while (!receiveComplete2){
        getTFminiData2(&distance2, &strength2, &receiveComplete2);
      }
      receiveComplete2 = false;
      
      while (Serial3.available()) {
        String x = Serial3.readStringUntil('\n');
        deg = x.toDouble();
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
        Serial.print("setting degree");
        Serial.println(deg);
        setdegree();
      }
      else {
        if (looptime <= intervalx) {
          Serial.print("setting x");
          Serial.println(distance1);
          setposx();
        }
        else {
          if (looptime <= intervaly) {
            Serial.print("setting y");
            Serial.println(distance2);
            setposy();
          }
          else {
            pidset = false;
          }
        }
      }
    }
 }
