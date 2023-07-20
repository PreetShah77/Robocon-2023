#include<Servo.h>
Servo esc1;
void setup() {
  // put your setup code here, to run once:
  esc1.attach(9);
  esc1.write(30);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  esc1.write(150);
 
}
