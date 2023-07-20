// #include <SoftwareSerial.h>  
//
//SoftwareSerial port(69,68);
// SoftwareSerial portOne(66, 67);
// SoftwareSerial portTwo(68,69); 


void getTFminiData2(int* distance, int* strength, boolean* complete) {
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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
//  Serial1.begin(115200);
//  Serial3.begin(115200);
//  port.begin(115200);
}

void loop() {
//  int distance1 = 0;
//  int strength1 = 0;
//  boolean receiveComplete1 = false;

  int distance2 = 0;
  int strength2 = 0;
  boolean receiveComplete2 = false;
//
//  while(!receiveComplete1) {
//    getTFminiData1(&port, &distance1, &strength1, &receiveComplete1);
//    if(receiveComplete1) {
//      Serial.print(distance1);
//      Serial.print("cm\t");
//      Serial.print("strength1: ");
//      Serial.print(strength1);
//      Serial.println("\t");
//    }
//  }
//  receiveComplete1 = false;

  while(!receiveComplete2) {
    getTFminiData2(&distance2, &strength2, &receiveComplete2);
    if(receiveComplete2) {
      Serial.print(distance2);
      Serial.print("cm\t");
      Serial.print("strength2: ");
      Serial.println(strength2);
    }
  }
  receiveComplete2 = false;
}
