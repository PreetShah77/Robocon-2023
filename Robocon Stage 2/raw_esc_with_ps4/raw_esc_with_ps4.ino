#include<Servo.h>
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];   
boolean newData = false;
int f_pwm, b_pwm, l_pwm, r_pwm, lr_pwm, rr_pwm;
int correction_factorright = 0;
int correction_factorleft = 0;
int robot_speed = 255;
int pwm;

Servo esc1;
Servo esc2;
char button_value[0];
unsigned int l_value = 0;
unsigned int r_value = 0;
signed int l_y_value = 0;
signed int r_x_value = 0;

void setup()
{
  Serial3.begin(115200);
  Serial.begin(115200);
  esc1.attach(9);
  esc2.attach(8);
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
    if(button_value[0] == 'A')
  {
    esc1.write(150);
    esc2.write(150);
  }
  if(button_value[0] == 'C')
  {
    esc1.write(30);
    esc2.write(30);
  }
    f_pwm = map(l_y_value, 10, 130, 10, robot_speed);
  b_pwm = map(l_y_value, -10, -130, 10, robot_speed);
  l_pwm = map(r_x_value, 10, 130, 10, robot_speed);
  r_pwm = map(r_x_value, -10, -130, 10, robot_speed);
  rr_pwm = map(r_value, 0, 255, 10, robot_speed);
  lr_pwm = map(l_value, 0, 255, 10, robot_speed);

    //analog movement
  if (l_value > 5) Left_rotate(lr_pwm); //l1 button
  else if (r_value > 5) Right_rotate(rr_pwm); //r1 button
  else if (r_x_value < -10) Right(r_pwm); //analog right hat
  else if (r_x_value > 10) Left(l_pwm); // analog right hat
  else if (l_y_value > 10) Forward(f_pwm); // analog left hat
  else if (l_y_value < -10) Backward(b_pwm); // analog left 
  else{
    Hold();
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
