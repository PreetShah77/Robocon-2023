#include <Servo.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <FastLED.h>

#define M1_pwm 5
#define M1_DIR 28
#define M2_pwm 4
#define M2_DIR 26
#define M3_pwm 3
#define M3_DIR 24
#define M4_pwm 2
#define M4_DIR 22
#define pick1_C 47
#define pick1_CC 49

CRGB leds[26];
Servo esc1;
Servo esc2;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];   
boolean newData = false;

char button_value[0];
unsigned int l_value = 0;
unsigned int r_value = 0;
signed int l_y_value = 0;
signed int r_x_value = 0;

int layout=0;
bool lay;

int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
int robot_speed = 255;

long positions[1];
const int stepPin1 = 41;
const int dirPin1 = 43;
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
int pwm;
bool temp=true;
bool yaw_reverse=false;
bool yaw_direct=false;
long double pitch[5] = { 18.6069, 23.5163, 20.0174, 22.4177, 1}; 
PID PIDX(&distance1, &outputx, &set_posx, kpx, kdx, kix, DIRECT);
PID PIDY(&distance2, &outputy, &set_posy, kpy, kdy, kiy, DIRECT);
PID PIDDEG(&deg, &outputd, &set_posd, kpd, kdd, kid, DIRECT);

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1,dirPin1,45);
int grab1=51;
int shoot1=53;
long angle;
void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  delay(20);
  Serial.begin(115200);
  delay(20);
  Serial2.begin(115200);
  delay(20);
  Serial3.begin(115200);
  delay(20);
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
  PIDX.SetOutputLimits(-200,200);
  PIDY.SetOutputLimits(-200,200);
  PIDDEG.SetOutputLimits(-200,200);
  esc1.attach(7);
  esc2.attach(6);
  esc1.write(30);
  esc2.write(30);
  delay(1000);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pick1_C,OUTPUT);
  pinMode(pick1_CC,OUTPUT);
  pinMode(grab1,OUTPUT);
  pinMode(shoot1,OUTPUT);
  stepper1.setMaxSpeed(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  ps4_data();

  //SPLITTING DATA OF PS4
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    split_data();
    newData = false;
  }
  move();
  if(button_value[0]=='I'){ digitalWrite(pick1_C,HIGH); digitalWrite(pick1_CC,LOW);}
    else if(button_value[0]=='A'){ digitalWrite(pick1_C,LOW); digitalWrite(pick1_CC,HIGH);}
    else{digitalWrite(pick1_C,LOW); digitalWrite(pick1_CC,LOW);}

    if(button_value[0]=='B')      digitalWrite(grab1,HIGH);
    else if(button_value[0]=='D') digitalWrite(grab1,LOW);
    else;
  
  if(button_value[0] == 'G') pidset=true; 
  else;
  if(button_value[0] == 'L') i=0;
  else;
  if(button_value[0] == 'F'){
      esc1.write(150);
      esc2.write(150);
    }
    else if(button_value[0] == 'H'){
      esc1.write(30);
      esc2.write(30);
    }
    else{}
    if(button_value[0] == 'g') digitalWrite(shoot1,HIGH);
    else digitalWrite(shoot1,LOW);
    
    if(button_value[0] == 'a'){
      // angle=pitch[0];
      // stepangle(angle);
      set_posd= -3.09; //POSITIVE 10
      yaw_direct=true;
    }
    else if(button_value[0] == 'b'){
      // angle=pitch[1];
      // stepangle(angle);
      set_posd= 43.36; //POSITIVE 10 // 45
      yaw_direct=true;
    }
    else if(button_value[0] == 'c'){
      // angle=pitch[2];
      // stepangle(angle);
      set_posd= 39.04; //POSITIVE 10 39.04
      yaw_direct=true;
    }
    else if(button_value[0] == 'd'){
      // angle=pitch[3];
      // stepangle(angle);
      set_posd= 67.29; //POSITIVE 10
      yaw_direct=true;
    }
    else if(button_value[0] == 'e'){
      // angle=pitch[4];
      // stepangle(angle);
      set_posd= 0; //POSITIVE 10
      yaw_direct=true;
    }
    else{}

  while (pidset == true) {
    
    while (!receiveComplete1) {
      getTFminiData1(&distance1, &strength1, &receiveComplete1);
    }
    receiveComplete1 = false;

    while (!receiveComplete2) {
      getTFminiData2(&distance2, &strength2, &receiveComplete2);
    }
    receiveComplete2 = false;
    
    get_degree();
    
    currenttime = millis();
    if (i == 0) {
      temptime = currenttime;
      i = 1;
    }
    looptime = currenttime - temptime;
    set_posd=0;
    if (looptime <= intervaldeg) { 
      setdegree();
      Serial.print("setting degree"); 
      Serial.println(deg);
    } 
    else{
     if (looptime <= intervalx){
        Serial.print("setting x");
        Serial.println(distance1);
        setposx();
      } 
    else{
       if (looptime <= intervaly){
          Serial.print("setting y");
          Serial.println(distance2);
          setposy();
        } 
        else{
          pidset = false;
        }
      }
    }
  }
  while(yaw_reverse==true){
      get_degree();
      currenttime = millis();
      if (i == 0) {
      temptime = currenttime;
      i = 1;
      }
      looptime = currenttime - temptime;
      PIDDEG.SetControllerDirection(REVERSE);
      if (looptime <= intervaldeg) { setdegree(); Serial.print("setting degree"); Serial.println(deg);}
      else yaw_reverse=false;
  }
  while(yaw_direct == true){
    get_degree();
      currenttime = millis();
      if (i == 0) {
      temptime = currenttime;
      i = 1;
      }
      looptime = currenttime - temptime;
      PIDDEG.SetControllerDirection(DIRECT);
      if (looptime <= intervalyaw) { setdegree(); Serial.print("setting degree"); Serial.println(deg);}
      else yaw_direct=false;
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
void getTFminiData1(double* distance1, double* strength1, bool* complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0; 
  static int rx[9];

  if(Serial.available()) {  
    rx[i] = Serial.read();
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance1 = rx[2] + rx[3] * 256;
        *strength1 = rx[4] + rx[5] * 256;
        *complete = true;
        Serial.println("hi1");
      }
      i = 0;
    } else {
      Serial.println("hi");
      i++;
    } 
  }  
}
void getTFminiData2(double* distance2, double* strength2, bool* complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0; 
  static int rx[9];

  if(Serial2.available()) {  
    rx[i] = Serial2.read();
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance2 = rx[2] + rx[3] * 256;
        *strength2 = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    } 
  }  
}

void move(){
  f_pwm = map(l_y_value, 10, 130, 10, robot_speed);
  b_pwm = map(l_y_value, -10, -130, 10, robot_speed);
  l_pwm = map(r_x_value, 10, 130, 10, robot_speed);
  r_pwm = map(r_x_value, -10, -130, 10, robot_speed);
  rr_pwm = map(r_value, 0, 255, 10, 170);
  lr_pwm = map(l_value, 0, 255, 10, 170);

    if (l_value > 10) Left_rotate(lr_pwm);
    else if (r_value > 10) Right_rotate(rr_pwm);
    else if (r_x_value < -10) Left(r_pwm);
    else if (r_x_value > 10) Right(l_pwm);
    else if (l_y_value > 10) Forward(f_pwm);
    else if (l_y_value < -10) Backward(b_pwm);
    else Hold();
}
void get_degree(){
  while (Serial1.available()) {
      String x = Serial1.readStringUntil('\n');
      deg = x.toDouble();
    }
}

void Forward(int pwm) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, HIGH);
  analogWrite(M4_pwm, pwm);//-abs(10*pwm/255)));
  analogWrite(M1_pwm, pwm);//-abs(4*pwm/255)));
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, pwm);//-abs(6*pwm/255)));
}

void Backward(int pwm) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, HIGH);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, pwm);//-abs(10*pwm/255)));
  analogWrite(M1_pwm, pwm);//-abs(4*pwm/255)));
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, pwm);//-abs(6*pwm/255)));
}

void Left(int pwm) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, HIGH);
  digitalWrite(M4_DIR, HIGH);
  analogWrite(M4_pwm, pwm);//-abs(10*pwm/255)));
  analogWrite(M1_pwm, pwm);//-abs(4*pwm/255)));
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, pwm);//-abs(6*pwm/255)));
}

void Right(int pwm) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, pwm);//-abs(10*pwm/255)));
  analogWrite(M1_pwm, pwm);//-abs(4*pwm/255)));
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm,pwm);//-abs(6*pwm/255)));
}

void Forward_left(int pwm) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, 0);
  analogWrite(M1_pwm, (pwm-abs(4*pwm/255)));
  analogWrite(M2_pwm, 0);
  analogWrite(M3_pwm, (pwm-abs(6*pwm/255)));
}

void Forward_right(int pwm) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, HIGH);
  analogWrite(M4_pwm, (pwm-abs(10*pwm/255)));
  analogWrite(M1_pwm, 0);
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, 0);
}

void Backward_left(int pwm) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, (pwm-abs(10*pwm/255)));
  analogWrite(M1_pwm, 0);
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, 0);
}

void Backward_right(int pwm) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, HIGH);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, 0);
  analogWrite(M1_pwm, (pwm-abs(4*pwm/255)));
  analogWrite(M2_pwm, 0);
  analogWrite(M3_pwm, (pwm-abs(6*pwm/255)));
}

void Right_rotate(int pwm) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, HIGH);
  digitalWrite(M4_DIR, HIGH);
  analogWrite(M4_pwm, pwm);//-abs(10*pwm/255)));
  analogWrite(M1_pwm, pwm);//-abs(4*pwm/255)));
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, pwm);//-abs(6*pwm/255)));
}

void Left_rotate(int pwm) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, pwm);//-abs(10*pwm/255)));
  analogWrite(M1_pwm, pwm);//-abs(4*pwm/255)));
  analogWrite(M2_pwm, pwm);
  analogWrite(M3_pwm, pwm);//-abs(6*pwm/255)));
}

void Hold() {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_pwm, 0);
  analogWrite(M1_pwm, 0);
  analogWrite(M2_pwm, 0);
  analogWrite(M3_pwm, 0);
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

//SPLIT PS4 DATA
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
